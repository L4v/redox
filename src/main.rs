use serde::{Deserialize, Serialize};
use serde_json;

use redox::{M44, V3};

fn main() {
    let m = M44::with_value(4.0);
    let serialized_m = serde_json::to_string(&m).unwrap();

    let t = V3::new(1.0, 2.0, 3.0);
    let translated = m.translate(t);
    let serialized_translated = serde_json::to_string(&translated).unwrap();
    println!("{serialized_m}");
    println!("{serialized_translated}");
}
