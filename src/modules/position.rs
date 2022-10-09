use super::{node::Node, ordered_float::OrderedFloat};

#[derive(Debug, PartialEq, Eq)]
pub enum Position {
    Open(Node),
    Weighted(Node, OrderedFloat),
    Walled,
}
