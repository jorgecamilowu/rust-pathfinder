use std::cmp::Ordering;
use std::ops::Add;

#[derive(PartialEq, PartialOrd, Debug, Clone, Copy)]
pub struct OrderedFloat(f64);

/**
 * DS to represent order in floats. Excludes NaNs
 */
impl OrderedFloat {
    pub fn new(val: f64) -> Option<OrderedFloat> {
        if val.is_nan() {
            None
        } else {
            Some(OrderedFloat(val))
        }
    }

    pub fn pow(&self, other: f64) -> OrderedFloat {
        OrderedFloat::new(f64::powf(self.0, other)).unwrap()
    }
}

impl Eq for OrderedFloat {}

impl Ord for OrderedFloat {
    fn cmp(&self, other: &OrderedFloat) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl Add for OrderedFloat {
    type Output = OrderedFloat;
    fn add(self, other: OrderedFloat) -> OrderedFloat {
        OrderedFloat::new(self.0 + other.0).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orders() {
        let mut vec: Vec<OrderedFloat> = vec![1.0, 0.7]
            .iter()
            .map(|v| OrderedFloat::new(*v).unwrap())
            .collect();
        vec.sort();
        assert_eq!(
            vec,
            [
                OrderedFloat::new(0.7).unwrap(),
                OrderedFloat::new(1.0).unwrap(),
            ]
        )
    }

    #[test]
    fn adds() {
        let first = OrderedFloat::new(1.5).unwrap();
        let second = OrderedFloat::new(1.7).unwrap();
        assert_eq!(first + second, OrderedFloat::new(3.2).unwrap());
    }
}
