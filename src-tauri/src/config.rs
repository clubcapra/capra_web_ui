use crate::capra::Config;
use bytes::Bytes;
use prost::Message;
use std::io::{self, Read};
use std::sync::Mutex;

#[derive(Default, serde::Serialize)]
pub struct ConnectionConfig {
    pub host: Mutex<Option<String>>,
    pub port: Mutex<Option<u16>>,
    pub robot_name: Mutex<Option<String>>,
}

pub fn get_config() -> Config {
    let buff = std::fs::File::open("config.bin").map(|mut file| {
        let mut buff = Vec::new();
        file.read_to_end(&mut buff).unwrap();
        Bytes::from(buff)
    });

    if let Ok(buff) = buff {
        return Config::decode(buff).unwrap();
    }

    return Config::default();
}

pub fn save_config(config: &Config) -> io::Result<()> {
    let buff = &mut Vec::new();

    config.encode(buff.into()).unwrap();
    std::fs::write("config.bin", buff)
}
