use super::{
    node::{Coordinate, Node},
    ordered_float::OrderedFloat,
};
use std::ops::Index;

#[derive(Debug, PartialEq, Eq)]
pub enum Position {
    Open(Node),
    Weighted(Node, OrderedFloat),
    Walled,
}

type Plane = Vec<Position>;

#[derive(Debug)]
pub struct Board {
    rows: usize,
    columns: usize,
    plane: Box<Plane>,
}

impl Index<Coordinate> for Board {
    type Output = Position;

    fn index(&self, (row, column): Coordinate) -> &Self::Output {
        &self.plane[self.columns * row + column]
    }
}

impl Board {
    pub fn new(rows: usize, columns: usize, plane: Plane) -> Board {
        Board {
            rows,
            columns,
            plane: Box::new(plane),
        }
    }

    pub fn get_plane(&self) -> &Plane {
        &self.plane
    }

    pub fn get_dimensions(&self) -> (usize, usize) {
        (self.rows, self.columns)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn get_board() -> Board {
        Board {
            rows: 2,
            columns: 2,
            plane: Box::new(vec![Position::Walled, Position::Open(Node::new((1, 1)))]),
        }
    }

    #[test]
    fn returns_dimensions() {
        let board = get_board();

        assert_eq!(board.get_dimensions(), (2, 2))
    }

    #[test]
    fn returns_plane() {
        let plane: Vec<Position> = vec![Position::Walled, Position::Open(Node::new((1, 1)))];

        let board = get_board();

        assert_eq!(*board.get_plane(), plane);
    }
}
