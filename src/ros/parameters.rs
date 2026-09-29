//! ROS 2 parameters: the six `rcl_interfaces` parameter services and
//! `/parameter_events`, so `ros2 param list|get|describe|dump|set` work
//! against the node. Parameters are statically typed like rclcpp's; setting
//! one needs a [`SetHandler`] and a parameter not marked read-only.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, PoisonError};

use super::context::Result;
use super::msgs::builtin_interfaces::Time;
use super::msgs::rcl_interfaces::{
    DescribeParameters, DescribeParametersResponse, GetParameterTypes, GetParameterTypesResponse,
    GetParameters, GetParametersResponse, ListParameters, ListParametersResponse,
    ListParametersResult, Parameter, ParameterDescriptor, ParameterEvent, ParameterValue,
    SetParameters, SetParametersAtomically, SetParametersAtomicallyResponse, SetParametersResponse,
    SetParametersResult,
};
use super::{Node, Publisher, QosProfile};

/// A parameter: value, description and whether it can be set.
#[derive(Debug, Clone, PartialEq)]
pub struct Param {
    /// Value.
    pub value: ParameterValue,
    /// Human-readable description.
    pub description: String,
    /// Refuse set requests.
    pub read_only: bool,
}

/// Converts a JSON value; `None` for null and non-homogeneous arrays.
#[must_use]
pub fn value_from_json(v: &serde_json::Value) -> Option<ParameterValue> {
    use serde_json::Value;
    let mut p = ParameterValue::default();
    match v {
        Value::Null | Value::Object(_) => return None,
        Value::Bool(b) => {
            p.r#type = ParameterValue::BOOL;
            p.bool_value = *b;
        }
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                p.r#type = ParameterValue::INTEGER;
                p.integer_value = i;
            } else {
                p.r#type = ParameterValue::DOUBLE;
                p.double_value = n.as_f64()?;
            }
        }
        Value::String(s) => {
            p.r#type = ParameterValue::STRING;
            p.string_value.clone_from(s);
        }
        Value::Array(items) => {
            if items.iter().all(Value::is_boolean) && !items.is_empty() {
                p.r#type = ParameterValue::BOOL_ARRAY;
                p.bool_array_value = items.iter().filter_map(Value::as_bool).collect();
            } else if items.iter().all(Value::is_i64) && !items.is_empty() {
                p.r#type = ParameterValue::INTEGER_ARRAY;
                p.integer_array_value = items.iter().filter_map(Value::as_i64).collect();
            } else if items.iter().all(Value::is_number) && !items.is_empty() {
                p.r#type = ParameterValue::DOUBLE_ARRAY;
                p.double_array_value = items.iter().filter_map(Value::as_f64).collect();
            } else if items.iter().all(Value::is_string) {
                p.r#type = ParameterValue::STRING_ARRAY;
                p.string_array_value = items
                    .iter()
                    .filter_map(|s| s.as_str().map(str::to_owned))
                    .collect();
            } else {
                // Arrays of tables: one JSON string each.
                p.r#type = ParameterValue::STRING_ARRAY;
                p.string_array_value = items.iter().map(Value::to_string).collect();
            }
        }
    }
    Some(p)
}

/// Converts a parameter value to JSON; `None` for an unset value or a
/// non-finite double.
#[must_use]
pub fn json_from_value(v: &ParameterValue) -> Option<serde_json::Value> {
    use serde_json::Value;
    let double = |d: f64| serde_json::Number::from_f64(d).map(Value::Number);
    Some(match v.r#type {
        ParameterValue::BOOL => Value::Bool(v.bool_value),
        ParameterValue::INTEGER => Value::from(v.integer_value),
        ParameterValue::DOUBLE => double(v.double_value)?,
        ParameterValue::STRING => Value::String(v.string_value.clone()),
        ParameterValue::BYTE_ARRAY => Value::from(v.byte_array_value.clone()),
        ParameterValue::BOOL_ARRAY => Value::from(v.bool_array_value.clone()),
        ParameterValue::INTEGER_ARRAY => Value::from(v.integer_array_value.clone()),
        ParameterValue::DOUBLE_ARRAY => Value::Array(
            v.double_array_value
                .iter()
                .map(|&d| double(d))
                .collect::<Option<_>>()?,
        ),
        ParameterValue::STRING_ARRAY => Value::from(v.string_array_value.clone()),
        _ => return None,
    })
}

/// Flattens a JSON object into dotted parameter names.
#[must_use]
pub fn flatten_json(prefix: &str, v: &serde_json::Value) -> BTreeMap<String, ParameterValue> {
    let mut out = BTreeMap::new();
    let mut stack = vec![(prefix.to_owned(), v)];
    while let Some((name, v)) = stack.pop() {
        if let serde_json::Value::Object(map) = v {
            for (k, child) in map {
                let child_name = if name.is_empty() {
                    k.clone()
                } else {
                    format!("{name}.{k}")
                };
                stack.push((child_name, child));
            }
        } else if let Some(p) = value_from_json(v) {
            out.insert(name, p);
        }
    }
    out
}

/// `rclcpp`'s `list_parameters` selection.
#[must_use]
pub fn list(names: &[String], prefixes: &[String], depth: u64) -> ListParametersResult {
    const SEPARATOR: char = '.';
    let depth_of = |s: &str| s.matches(SEPARATOR).count() as u64;
    let mut result = ListParametersResult::default();
    for name in names {
        let get_all = prefixes.is_empty() && (depth == 0 || depth_of(name) < depth);
        let prefix_matches = prefixes.iter().any(|prefix| {
            if name == prefix {
                return true;
            }
            // Like rclcpp and rclpy: the separators after `prefix.`.
            name.strip_prefix(prefix.as_str())
                .and_then(|rest| rest.strip_prefix(SEPARATOR))
                .is_some_and(|rest| depth == 0 || depth_of(rest) < depth)
        });
        if get_all || prefix_matches {
            result.names.push(name.clone());
            if let Some((p, _)) = name.rsplit_once(SEPARATOR)
                && !result.prefixes.iter().any(|x| x == p)
            {
                result.prefixes.push(p.to_owned());
            }
        }
    }
    result
}

/// Validates and applies accepted parameter changes; `Err` carries the
/// reason reported to the caller, and nothing is stored.
pub type SetHandler = Arc<dyn Fn(&[Parameter]) -> std::result::Result<(), String> + Send + Sync>;

/// The parameter services of a node; drop to withdraw them.
pub struct ParameterServer {
    _list: super::ServiceServer<ListParameters>,
    _get: super::ServiceServer<GetParameters>,
    _types: super::ServiceServer<GetParameterTypes>,
    _describe: super::ServiceServer<DescribeParameters>,
    _set: super::ServiceServer<SetParameters>,
    _set_atomically: super::ServiceServer<SetParametersAtomically>,
    _events: Arc<Publisher<ParameterEvent>>,
}

impl std::fmt::Debug for ParameterServer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParameterServer").finish_non_exhaustive()
    }
}

/// The parameters `names`, or `None` when one is undeclared: rclcpp's
/// get, get-types and describe services then answer with an empty list.
fn known<'a>(
    params: &'a BTreeMap<String, Param>,
    names: &'a [String],
) -> Option<impl Iterator<Item = &'a Param> + 'a> {
    names
        .iter()
        .all(|n| params.contains_key(n))
        .then(|| names.iter().filter_map(|n| params.get(n)))
}

/// rclcpp's name for a parameter type.
fn type_name(t: u8) -> &'static str {
    match t {
        ParameterValue::BOOL => "bool",
        ParameterValue::INTEGER => "integer",
        ParameterValue::DOUBLE => "double",
        ParameterValue::STRING => "string",
        ParameterValue::BYTE_ARRAY => "byte_array",
        ParameterValue::BOOL_ARRAY => "bool_array",
        ParameterValue::INTEGER_ARRAY => "integer_array",
        ParameterValue::DOUBLE_ARRAY => "double_array",
        ParameterValue::STRING_ARRAY => "string_array",
        _ => "not set",
    }
}

fn is_empty_array(v: &ParameterValue) -> bool {
    match v.r#type {
        ParameterValue::BYTE_ARRAY => v.byte_array_value.is_empty(),
        ParameterValue::BOOL_ARRAY => v.bool_array_value.is_empty(),
        ParameterValue::INTEGER_ARRAY => v.integer_array_value.is_empty(),
        ParameterValue::DOUBLE_ARRAY => v.double_array_value.is_empty(),
        ParameterValue::STRING_ARRAY => v.string_array_value.is_empty(),
        _ => false,
    }
}

fn failure(reason: String) -> SetParametersResult {
    SetParametersResult {
        successful: false,
        reason,
    }
}

/// Checks a request like rclcpp's statically typed parameters and returns
/// the values to store. An empty array of any element type is accepted for
/// an array parameter: the `ros2 param set` CLI types `[]` as a bool array.
fn check(
    params: &BTreeMap<String, Param>,
    request: &[Parameter],
) -> std::result::Result<Vec<Parameter>, String> {
    let mut accepted = Vec::with_capacity(request.len());
    for p in request {
        let Some(current) = params.get(&p.name) else {
            return Err(format!(
                "parameter '{}' cannot be set because it was not declared",
                p.name
            ));
        };
        if current.read_only {
            return Err(format!("Trying to set a read-only parameter: {}.", p.name));
        }
        let (have, want) = (current.value.r#type, p.value.r#type);
        if want == ParameterValue::NOT_SET {
            return Err(format!(
                "cannot undeclare a statically typed parameter: {}",
                p.name
            ));
        }
        let value = if have == want {
            p.value.clone()
        } else if is_empty_array(&p.value) && have >= ParameterValue::BYTE_ARRAY {
            ParameterValue {
                r#type: have,
                ..ParameterValue::default()
            }
        } else {
            return Err(format!(
                "Wrong parameter type, parameter {{{}}} is of type {{{}}}, setting it to {{{}}} is not allowed.",
                p.name,
                type_name(have),
                type_name(want)
            ));
        };
        accepted.push(Parameter {
            name: p.name.clone(),
            value,
        });
    }
    Ok(accepted)
}

/// Applies `request` all or nothing: checks it, lets `on_set` validate and
/// act on it, stores it and announces it on `/parameter_events`.
fn apply(
    state: &Mutex<BTreeMap<String, Param>>,
    on_set: Option<&SetHandler>,
    events: &Publisher<ParameterEvent>,
    node: &str,
    request: &[Parameter],
) -> SetParametersResult {
    // Held throughout, so concurrent requests apply one at a time.
    let mut params = state.lock().unwrap_or_else(PoisonError::into_inner);
    let accepted = match check(&params, request) {
        Ok(accepted) => accepted,
        Err(reason) => return failure(reason),
    };
    if accepted.is_empty() {
        return SetParametersResult {
            successful: true,
            reason: String::new(),
        };
    }
    let Some(on_set) = on_set else {
        return failure("parameters cannot be changed at run time".into());
    };
    if let Err(reason) = on_set(&accepted) {
        return failure(reason);
    }
    for p in &accepted {
        if let Some(slot) = params.get_mut(&p.name) {
            slot.value = p.value.clone();
        }
    }
    // Published before the lock is released, so events keep the order in
    // which the changes were applied.
    let _ = events.publish(&ParameterEvent {
        stamp: Time::now(),
        node: node.to_owned(),
        changed_parameters: accepted,
        ..ParameterEvent::default()
    });
    drop(params);
    SetParametersResult {
        successful: true,
        reason: String::new(),
    }
}

impl ParameterServer {
    /// Serves `params` on `node` and announces them on `/parameter_events`.
    /// Parameters not marked read-only can be set when `on_set` is given;
    /// it sees each accepted change before it is stored.
    ///
    /// # Errors
    /// Zenoh or naming errors.
    pub fn new(
        node: &Node,
        params: BTreeMap<String, Param>,
        on_set: Option<SetHandler>,
    ) -> Result<Self> {
        let names: Arc<Vec<String>> = Arc::new(params.keys().cloned().collect());
        // rclcpp's ParameterEventsQoS: keep last 1000, reliable, volatile.
        let events = Arc::new(node.create_publisher::<ParameterEvent>(
            "/parameter_events",
            QosProfile::keep_last(1000),
        )?);
        let fqn = node.fully_qualified_name();
        let _ = events.publish(&ParameterEvent {
            stamp: Time::now(),
            node: fqn.clone(),
            new_parameters: params
                .iter()
                .map(|(name, p)| Parameter {
                    name: name.clone(),
                    value: p.value.clone(),
                })
                .collect(),
            ..ParameterEvent::default()
        });
        let state = Arc::new(Mutex::new(params));

        let n = Arc::clone(&names);
        let list =
            node.create_service::<ListParameters, _>("~/list_parameters", move |req, _| {
                ListParametersResponse {
                    result: list(&n, &req.prefixes, req.depth),
                }
            })?;
        let s = Arc::clone(&state);
        let get = node.create_service::<GetParameters, _>("~/get_parameters", move |req, _| {
            let p = s.lock().unwrap_or_else(PoisonError::into_inner);
            GetParametersResponse {
                values: known(&p, &req.names)
                    .map(|found| found.map(|x| x.value.clone()).collect())
                    .unwrap_or_default(),
            }
        })?;
        let s = Arc::clone(&state);
        let types =
            node.create_service::<GetParameterTypes, _>("~/get_parameter_types", move |req, _| {
                let p = s.lock().unwrap_or_else(PoisonError::into_inner);
                GetParameterTypesResponse {
                    types: known(&p, &req.names)
                        .map(|found| found.map(|x| x.value.r#type).collect())
                        .unwrap_or_default(),
                }
            })?;
        let s = Arc::clone(&state);
        let describe = node.create_service::<DescribeParameters, _>(
            "~/describe_parameters",
            move |req, _| {
                let p = s.lock().unwrap_or_else(PoisonError::into_inner);
                DescribeParametersResponse {
                    descriptors: known(&p, &req.names)
                        .map(|found| {
                            found
                                .zip(&req.names)
                                .map(|(x, name)| ParameterDescriptor {
                                    name: name.clone(),
                                    r#type: x.value.r#type,
                                    description: x.description.clone(),
                                    read_only: x.read_only,
                                    ..ParameterDescriptor::default()
                                })
                                .collect()
                        })
                        .unwrap_or_default(),
                }
            },
        )?;
        let setter = {
            let (state, events, fqn) = (Arc::clone(&state), Arc::clone(&events), fqn.clone());
            move |request: &[Parameter]| apply(&state, on_set.as_ref(), &events, &fqn, request)
        };
        let setter = Arc::new(setter);
        let apply_one = Arc::clone(&setter);
        // Like rclcpp: each parameter is applied on its own.
        let set = node.create_service::<SetParameters, _>("~/set_parameters", move |req, _| {
            SetParametersResponse {
                results: req
                    .parameters
                    .iter()
                    .map(|p| apply_one(std::slice::from_ref(p)))
                    .collect(),
            }
        })?;
        let set_atomically = node.create_service::<SetParametersAtomically, _>(
            "~/set_parameters_atomically",
            move |req, _| SetParametersAtomicallyResponse {
                result: setter(&req.parameters),
            },
        )?;
        Ok(Self {
            _list: list,
            _get: get,
            _types: types,
            _describe: describe,
            _set: set,
            _set_atomically: set_atomically,
            _events: events,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flattening_and_types() {
        let v = serde_json::json!({
            "a": {"b": 1, "c": 2.5, "d": "x", "e": [1, 2], "f": [], "g": null, "h": [{"k": 1}]},
            "z": true
        });
        let f = flatten_json("", &v);
        assert_eq!(f["a.b"].r#type, ParameterValue::INTEGER);
        assert_eq!(f["a.c"].r#type, ParameterValue::DOUBLE);
        assert_eq!(f["a.d"].string_value, "x");
        assert_eq!(f["a.e"].integer_array_value, vec![1, 2]);
        assert_eq!(f["a.f"].r#type, ParameterValue::STRING_ARRAY);
        assert!(!f.contains_key("a.g"));
        assert_eq!(f["a.h"].string_array_value, vec!["{\"k\":1}".to_owned()]);
        assert!(f["z"].bool_value);
    }

    #[test]
    fn undeclared_names_empty_the_answer() {
        let mut params = BTreeMap::new();
        params.insert(
            "a".to_owned(),
            Param {
                value: ParameterValue::default(),
                description: String::new(),
                read_only: true,
            },
        );
        let ok = ["a".to_owned()];
        assert_eq!(known(&params, &ok).map(Iterator::count), Some(1));
        let bad = ["a".to_owned(), "b".to_owned()];
        assert!(known(&params, &bad).is_none());
    }

    #[test]
    fn listing_like_rclcpp() {
        let names: Vec<String> = ["use_sim_time", "ros.node_name", "ros.stream_qos.depth"]
            .map(String::from)
            .to_vec();
        let all = list(&names, &[], 0);
        assert_eq!(all.names.len(), 3);
        assert_eq!(
            all.prefixes,
            vec!["ros".to_owned(), "ros.stream_qos".to_owned()]
        );
        let top = list(&names, &[], 1);
        assert_eq!(top.names, vec!["use_sim_time".to_owned()]);
        // Depth 1 below a prefix lists its direct children.
        let ros = list(&names, &["ros".into()], 1);
        assert_eq!(ros.names, vec!["ros.node_name".to_owned()]);
        assert_eq!(list(&names, &["ros".into()], 2).names.len(), 2);
        assert!(list(&names, &["ro".into()], 0).names.is_empty());
        let ros_all = list(&names, &["ros".into()], 0);
        assert_eq!(ros_all.names.len(), 2);
    }

    fn double(d: f64) -> ParameterValue {
        ParameterValue {
            r#type: ParameterValue::DOUBLE,
            double_value: d,
            ..ParameterValue::default()
        }
    }

    fn set(name: &str, value: ParameterValue) -> Parameter {
        Parameter {
            name: name.into(),
            value,
        }
    }

    #[test]
    fn set_requests_are_checked_like_rclcpp() {
        let mut params = BTreeMap::new();
        let param = |value, read_only| Param {
            value,
            description: String::new(),
            read_only,
        };
        params.insert("rate".to_owned(), param(double(1.0), false));
        params.insert("fixed".to_owned(), param(double(1.0), true));
        let list = ParameterValue {
            r#type: ParameterValue::STRING_ARRAY,
            string_array_value: vec!["a".into()],
            ..ParameterValue::default()
        };
        params.insert("list".to_owned(), param(list, false));

        assert_eq!(
            check(&params, &[set("rate", double(2.0))]).unwrap()[0].value,
            double(2.0)
        );
        let integer = ParameterValue {
            r#type: ParameterValue::INTEGER,
            integer_value: 2,
            ..ParameterValue::default()
        };
        assert_eq!(
            check(&params, &[set("rate", integer)]).unwrap_err(),
            "Wrong parameter type, parameter {rate} is of type {double}, setting it to {integer} is not allowed."
        );
        assert_eq!(
            check(&params, &[set("fixed", double(2.0))]).unwrap_err(),
            "Trying to set a read-only parameter: fixed."
        );
        assert!(
            check(&params, &[set("missing", double(2.0))])
                .unwrap_err()
                .contains("not declared")
        );
        assert!(check(&params, &[set("rate", ParameterValue::default())]).is_err());
        // `ros2 param set node list "[]"` sends an empty bool array.
        let empty = ParameterValue {
            r#type: ParameterValue::BOOL_ARRAY,
            ..ParameterValue::default()
        };
        let cleared = check(&params, &[set("list", empty)]).unwrap();
        assert_eq!(cleared[0].value.r#type, ParameterValue::STRING_ARRAY);
        assert!(cleared[0].value.string_array_value.is_empty());
        // One bad parameter rejects the whole request.
        assert!(
            check(
                &params,
                &[set("rate", double(2.0)), set("fixed", double(2.0))]
            )
            .is_err()
        );
    }

    #[test]
    fn json_round_trip() {
        let v =
            serde_json::json!({"a": 1, "b": 2.5, "c": "x", "d": [1.0, 2.5], "e": ["p"], "f": true});
        for (_, p) in flatten_json("", &v) {
            let back = json_from_value(&p).unwrap();
            assert_eq!(value_from_json(&back).unwrap(), p);
        }
        assert!(json_from_value(&double(f64::NAN)).is_none());
        assert!(json_from_value(&ParameterValue::default()).is_none());
    }
}
