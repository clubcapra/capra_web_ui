use prost_build;
use std::env;

fn main() {
    tauri_build::build();
    env::set_var("OUT_DIR", "src/");
    let result = prost_build::compile_protos(&["src/protos/CapraMessages.proto"], &["src"]);
    match result {
        Ok(_) => {}
        Err(e) => panic!("Failed to compile protos: {}", e),
    }
}
