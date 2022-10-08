#[derive(Hash, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Node {
    position: Coordinate,
}

impl Node {
    pub fn new(position: Coordinate) -> Node {
        Node { position }
    }

    pub fn get_position(&self) -> Coordinate {
        self.position
    }
}

pub type Coordinate = (usize, usize);
