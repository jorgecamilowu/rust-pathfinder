use super::node::Node;

#[derive(Debug, PartialEq, Eq)]
pub enum Position {
    Open(Node),
    Weighted(Node, i32),
    Walled,
}
