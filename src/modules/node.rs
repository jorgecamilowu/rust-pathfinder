#[derive(Hash, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Node {
    position: Coordinate,
    cost: u32,
}

impl Node {
    pub fn new(position: Coordinate, cost: u32) -> Node {
        Node { position, cost }
    }

    pub fn get_position(&self) -> Coordinate {
        self.position
    }

    pub fn get_cost(&self) -> u32 {
        self.cost
    }
}

pub type Coordinate = (usize, usize);
