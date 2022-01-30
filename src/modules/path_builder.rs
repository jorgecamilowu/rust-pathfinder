use super::node::Coordinate;
use std::collections::HashMap;

pub struct PathBuilder {
    pub node_origins: HashMap<Coordinate, Coordinate>,
}

impl PathBuilder {
    pub fn new() -> PathBuilder {
        PathBuilder {
            node_origins: HashMap::new(),
        }
    }

    pub fn build(&self, goal: Coordinate) -> Option<Vec<Coordinate>> {
        let mut output: Vec<Coordinate> = Vec::new();
        let mut node = goal;

        output.push(goal);

        while self.node_origins.contains_key(&node) {
            node = *self.node_origins.get(&node).unwrap();
            output.push(node);
        }

        if output.len() < 1 {
            return None;
        }

        output.reverse();
        Some(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn it_builds_path() {
        let mut builder = PathBuilder::new();
        builder.node_origins.insert((4, 4), (3, 3));
        builder.node_origins.insert((3, 3), (2, 2));
        builder.node_origins.insert((2, 2), (1, 1));

        assert_eq!(
            Some(vec![(1, 1), (2, 2), (3, 3), (4, 4)]),
            builder.build((4, 4))
        );
    }
}
