use super::board::Board;
use super::node::{Coordinate, Node};
use super::ordered_float::OrderedFloat;
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
    heap: KeyedPriorityQueue<Coordinate, Reverse<OrderedFloat>>,
    // for a position x,  g_score[x] is the current shortest path from start to x
    g_score: HashMap<Coordinate, OrderedFloat>,
    // optimization, skip already expanded positions
    seen: HashSet<Coordinate>,
    board: Board,
}

type DiagonalIdentifiedPosition<'a> = (&'a Position, bool);

impl PathFinder {
    pub fn new(board: Board, path_builder: PathBuilder) -> PathFinder {
        PathFinder {
            board,
            path_builder,
            heap: KeyedPriorityQueue::<Coordinate, Reverse<OrderedFloat>>::new(),
            g_score: HashMap::new(),
            seen: HashSet::new(),
        }
    }

    // instead of manually checking the directions, traverse a vector of
    // direction offsets and validate oob with offsets. Haven't found a way
    // to handle usize and isize (the indexing operations can only happen with usize)
    // but offsets need to include negatives so at the very least it has to be isize
    // const STRAIGHT: [(isize, isize); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
    // const DIAGONALS: [(isize, isize); 4] = [(-1, -1), (-1, 1), (1, -1), (1, 1)];
    // const DIRECTIONS: [(isize, isize); 8] = [
    //     (-1, 0), (1, 0), (0, -1), (0, 1),
    //     (-1, -1), (-1, 1), (1, -1), (1, 1)
    // ];
    fn get_neighbors(node: Coordinate, board: &Board) -> Vec<DiagonalIdentifiedPosition> {
        let (board_rows, board_columns) = board.get_dimensions();
        let mut neighbors: Vec<DiagonalIdentifiedPosition> = Vec::with_capacity(8);
        let (row, column) = node;

        // top
        if row > 0 {
            neighbors.push((&board[(row - 1, column)], false))
        }

        // right
        if column < board_columns - 1 {
            // self.push_position((row, column + 1, false), &mut neighbors)
            neighbors.push((&board[(row, column + 1)], false))
        }

        // bottom
        if row < board_rows - 1 {
            // self.push_position((row + 1, column, false), &mut neighbors)
            neighbors.push((&board[(row + 1, column)], false))
        }

        // left
        if column > 0 {
            // self.push_position((row, column - 1, false), &mut neighbors)
            neighbors.push((&board[(row, column - 1)], false))
        }

        // top-left
        if row > 0 && column > 0 {
            // self.push_position((row - 1, column - 1, true), &mut neighbors)
            neighbors.push((&board[(row - 1, column - 1)], true))
        }

        // top-right
        if row > 0 && column < board_columns - 1 {
            // self.push_position((row - 1, column + 1, true), &mut neighbors)
            neighbors.push((&board[(row - 1, column + 1)], true))
        }

        // bottom-right
        if row < board_rows - 1 && column < board_columns - 1 {
            // self.push_position((row + 1, column + 1, true), &mut neighbors)
            neighbors.push((&board[(row + 1, column + 1)], true))
        }

        // bottom-left
        if row < board_rows - 1 && column > 0 {
            // self.push_position((row + 1, column - 1, true), &mut neighbors)
            neighbors.push((&board[(row + 1, column - 1)], true))
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
        let starting_distance = OrderedFloat::new(0.0).unwrap();

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
            for (position, is_diagonal) in Self::get_neighbors(current_position, &self.board) {
                match position {
                    Position::Open(node) | Position::Weighted(node, _)
                        if !self.seen.contains(&node.get_position()) =>
                    {
                        let (next_row, next_col) = node.get_position();
                        let weight = if let Position::Weighted(_, weight) = position {
                            *weight
                        } else {
                            OrderedFloat::new(0.0).unwrap()
                        };

                        let distance_to_next_node = {
                            if is_diagonal {
                                OrderedFloat::new(2.0).unwrap().pow(0.5) + weight
                            } else {
                                OrderedFloat::new(1.0).unwrap() + weight
                            }
                        };

                        let tentative_gscore =
                            self.g_score[&current_position] + distance_to_next_node;
                        let next_position = (next_row, next_col);
                        let new_fscore = tentative_gscore
                            + self.calculate_heuristic(next_position, goal.get_position());

                        match self.heap.entry(next_position) {
                            // path relaxation attempt:
                            // case where we had a previous record of a position's shortest path. Re-evaluate if current path is shorter
                            keyed_priority_queue::Entry::Occupied(entry)
                                if tentative_gscore < self.g_score[&next_position] =>
                            {
                                // mark the position's origin
                                self.path_builder
                                    .node_origins
                                    .insert(next_position, current_position);

                                // update f_score
                                entry.set_priority(Reverse(new_fscore));

                                // update g_score
                                self.g_score.insert(next_position, tentative_gscore);
                            }

                            // case where it is the first time we expand this position. Record it.
                            keyed_priority_queue::Entry::Vacant(entry) => {
                                // mark the position's origin
                                self.path_builder
                                    .node_origins
                                    .insert(next_position, current_position);

                                // update f_score
                                entry.set_priority(Reverse(new_fscore));

                                // update g_score
                                self.g_score.insert(next_position, tentative_gscore);
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

    // TODO refactor with strategy pattern
    // approximates distance of two coordinates using the Euclidean distance
    fn calculate_heuristic(&self, a: Coordinate, b: Coordinate) -> OrderedFloat {
        let (row_a, col_a) = a;
        let (row_b, col_b) = b;

        let dx = isize::pow(row_a as isize - row_b as isize, 2);
        let dy = isize::pow(col_a as isize - col_b as isize, 2);

        // casting isize to f64 is safe because we are using the squared difference -> dx and dy are always positive
        let d = f64::powf((dx + dy) as f64, 0.5);

        OrderedFloat::new(d).unwrap()
    }

    fn set_board(&mut self, board: Board) {
        self.board = board;
        self.cleanup();
    }

    fn cleanup(&mut self) {
        self.heap = KeyedPriorityQueue::<Coordinate, Reverse<OrderedFloat>>::new();
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
                (&Position::Open(Node::new((0, 1))), false),
                (&Position::Open(Node::new((1, 2))), false),
                (&Position::Open(Node::new((2, 1))), false),
                (&Position::Open(Node::new((1, 0))), false),
                (&Position::Open(Node::new((0, 0))), true),
                (&Position::Open(Node::new((0, 2))), true),
                (&Position::Open(Node::new((2, 2))), true),
                (&Position::Open(Node::new((2, 0))), true),
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
                ((&Position::Open(Node::new((0, 1)))), false),
                ((&Position::Open(Node::new((1, 0)))), false),
                ((&Position::Open(Node::new((1, 1)))), true),
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
                ((&Position::Open(Node::new((1, 2)))), false),
                ((&Position::Open(Node::new((2, 1)))), false),
                ((&Position::Open(Node::new((1, 1)))), true),
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
