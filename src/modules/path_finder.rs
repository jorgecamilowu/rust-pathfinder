use super::board::Board;
use super::node::{Coordinate, Node};
use super::path_builder::PathBuilder;
use super::position::Position;
use keyed_priority_queue::KeyedPriorityQueue;
use std::cmp::Reverse;
use std::collections::{HashMap, HashSet};

/**
 * Path finder using the A * Search algorithm.
 */
pub struct PathFinder {
    path_builder: PathBuilder,
    /**
     * Keyed by the f_score of a position.
     * f_score[x] represents our current best guess as to how short a path
     * from start to finish can be if it goes through n.
     * for position x, f_score[x] := g_score[x] + heuristic(x).
     */
    heap: KeyedPriorityQueue<Coordinate, Reverse<i32>>,
    // for a position x,  g_score[x] is the current shortest path from start to x
    g_score: HashMap<Coordinate, i32>,
    // optimization, skip already expanded positions
    seen: HashSet<Coordinate>,
    board: Board,
}

impl PathFinder {
    pub fn new(board: Board, path_builder: PathBuilder) -> PathFinder {
        PathFinder {
            board,
            path_builder,
            heap: KeyedPriorityQueue::<Coordinate, Reverse<i32>>::new(),
            g_score: HashMap::new(),
            seen: HashSet::new(),
        }
    }

    fn get_neighbors(node: Coordinate, board: &Board) -> Vec<&Position> {
        let (row_limit, col_limit) = board.get_dimensions();
        let mut neighbors: Vec<&Position> = Vec::with_capacity(8);

        let directions = vec![
            (-1isize, 0isize),
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, -1),
            (-1, 1),
            (1, 1),
            (1, -1),
        ];

        for (row_offset, col_offset) in directions {
            let (row, col) = node;

            let (row, col) = (row as isize + row_offset, col as isize + col_offset);

            if row >= 0 && row < row_limit as isize && col >= 0 && col < col_limit as isize {
                neighbors.push(&board[(row as usize, col as usize)]);
            }
        }

        neighbors
    }

    pub fn find_shortest_path(&mut self, start: &Node, goal: &Node) -> Option<Vec<Coordinate>> {
        let start_position = start.get_position();

        // skip if starting position is walled
        if let Position::Walled = self.board[start_position] {
            return None;
        }

        if start == goal {
            return Some(vec![start_position]);
        }

        // bootstrapping
        let starting_distance = 0;

        // Reverse wrapper ensures we use a min heap rather than the default max heap
        self.heap.push(start_position, Reverse(starting_distance));
        self.g_score.insert(start_position, starting_distance);

        while let Some((current_position, _)) = self.heap.pop() {
            // found goal
            if current_position == goal.get_position() {
                return self.path_builder.build(current_position);
            }

            // record already explored positions
            self.seen.insert(current_position);

            // explore neighbors
            for position in Self::get_neighbors(current_position, &self.board) {
                let weight = match position {
                    Position::Weighted(_, weight) => *weight,
                    _ => 0,
                };

                match position {
                    Position::Open(node) | Position::Weighted(node, _)
                        if !self.seen.contains(&node.get_position()) =>
                    {
                        let (next_row, next_col) = node.get_position();

                        let distance_to_next_node = 1 + weight;

                        let tentative_g_score =
                            self.g_score[&current_position] + distance_to_next_node;

                        let next_position = (next_row, next_col);

                        let new_f_score = tentative_g_score
                            + self.calculate_heuristic(next_position, goal.get_position());

                        match self.heap.entry(next_position) {
                            // path relaxation attempt:
                            // case where we had a previous record of a position's shortest path. Re-evaluate if current path is shorter
                            keyed_priority_queue::Entry::Occupied(entry)
                                if tentative_g_score < self.g_score[&next_position] =>
                            {
                                // mark the position's origin
                                self.path_builder
                                    .node_origins
                                    .insert(next_position, current_position);

                                // update f_score
                                entry.set_priority(Reverse(new_f_score));

                                // update g_score
                                self.g_score.insert(next_position, tentative_g_score);
                            }

                            // case where it is the first time we expand this position. Record it.
                            keyed_priority_queue::Entry::Vacant(entry) => {
                                // mark the position's origin
                                self.path_builder
                                    .node_origins
                                    .insert(next_position, current_position);

                                // update f_score
                                entry.set_priority(Reverse(new_f_score));

                                // update g_score
                                self.g_score.insert(next_position, tentative_g_score);
                            }
                            _ => {}
                        }
                    }
                    _ => {
                        // skip already explored or walled positions
                        continue;
                    }
                }
            }
        }

        None
    }

    // approximates distance of two coordinates using the Manhattan Distance
    fn calculate_heuristic(&self, a: Coordinate, b: Coordinate) -> i32 {
        const D1: isize = 1;
        const D2: isize = 1;

        let (row_a, col_a) = a;
        let (row_b, col_b) = b;

        let dx = row_a as isize - row_b as isize;
        let dy = col_a as isize - col_b as isize;

        (D1 * (dx + dy) + (D2 - 2 * D1) * dx.min(dy)) as i32
    }

    fn set_board(&mut self, board: Board) {
        self.board = board;
        self.cleanup();
    }

    fn cleanup(&mut self) {
        self.heap = KeyedPriorityQueue::<Coordinate, Reverse<i32>>::new();
        self.g_score = HashMap::new();
        self.seen = HashSet::new();
        self.path_builder = PathBuilder::new();
    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn build_path_finder() -> PathFinder {
        PathFinder::new(
            Board::new(
                3,
                3,
                vec![
                    Position::Open(Node::new((0, 0))),
                    Position::Open(Node::new((0, 1))),
                    Position::Open(Node::new((0, 2))),
                    Position::Open(Node::new((1, 0))),
                    Position::Open(Node::new((1, 1))),
                    Position::Open(Node::new((1, 2))),
                    Position::Open(Node::new((2, 0))),
                    Position::Open(Node::new((2, 1))),
                    Position::Open(Node::new((2, 2))),
                ],
            ),
            PathBuilder::new(),
        )
    }

    #[test]
    fn it_returns_all_neighbors() {
        let path_finder = build_path_finder();
        let neighbors = PathFinder::get_neighbors((1, 1), &path_finder.board);
        assert_eq!(
            neighbors,
            vec![
                &Position::Open(Node::new((0, 1))),
                &Position::Open(Node::new((1, 2))),
                &Position::Open(Node::new((2, 1))),
                &Position::Open(Node::new((1, 0))),
                &Position::Open(Node::new((0, 0))),
                &Position::Open(Node::new((0, 2))),
                &Position::Open(Node::new((2, 2))),
                &Position::Open(Node::new((2, 0))),
            ]
        );
    }

    #[test]
    fn it_returns_top_left_mid() {
        let path_finder = build_path_finder();
        let neighbors = PathFinder::get_neighbors((0, 0), &path_finder.board);
        assert_eq!(
            neighbors,
            vec![
                &Position::Open(Node::new((0, 1))),
                &Position::Open(Node::new((1, 0))),
                &Position::Open(Node::new((1, 1))),
            ]
        );
    }

    #[test]
    fn it_returns_left_bottom_mid() {
        let path_finder = build_path_finder();
        let neighbors = PathFinder::get_neighbors((2, 2), &path_finder.board);
        assert_eq!(
            neighbors,
            vec![
                &Position::Open(Node::new((1, 2))),
                &Position::Open(Node::new((2, 1))),
                &Position::Open(Node::new((1, 1))),
            ]
        );
    }

    #[test]
    fn it_calculates_heuristics() {
        let path_finder = build_path_finder();
        let closer = path_finder.calculate_heuristic((1, 1), (0, 0));
        let further = path_finder.calculate_heuristic((2, 2), (0, 0));

        assert!(closer < further);
    }

    #[test]
    fn it_computes_path() {
        let mut path_finder = PathFinder::new(
            Board::new(
                3,
                3,
                vec![
                    Position::Open(Node::new((0, 0))),
                    Position::Open(Node::new((0, 1))),
                    Position::Open(Node::new((0, 2))),
                    Position::Open(Node::new((1, 0))),
                    Position::Walled,
                    Position::Open(Node::new((1, 2))),
                    Position::Open(Node::new((2, 0))),
                    Position::Open(Node::new((2, 1))),
                    Position::Open(Node::new((2, 2))),
                ],
            ),
            PathBuilder::new(),
        );

        let result = path_finder.find_shortest_path(&Node::new((0, 0)), &Node::new((2, 1)));
        assert_eq!(result, Some(vec![(0, 0), (1, 0), (2, 1)]));
    }

    #[test]
    fn it_handles_non_existing_paths() {
        let mut path_finder = PathFinder::new(
            Board::new(
                3,
                3,
                vec![
                    Position::Open(Node::new((0, 0))),
                    Position::Walled,
                    Position::Open(Node::new((0, 2))),
                    Position::Open(Node::new((1, 0))),
                    Position::Walled,
                    Position::Open(Node::new((1, 2))),
                    Position::Open(Node::new((2, 0))),
                    Position::Walled,
                    Position::Open(Node::new((2, 2))),
                ],
            ),
            PathBuilder::new(),
        );

        let result = path_finder.find_shortest_path(&Node::new((0, 0)), &Node::new((2, 1)));
        assert_eq!(result, None);
    }
}
