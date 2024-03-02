// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::sync::Mutex;

mod capra;
mod config;

#[tauri::command]
fn set_state(host: String, port: u16, _name: String) {
    println!("Updating state - server");

    let mut config = config::get_config();
    let mut h = match config.host {
        Some(h) => h,
        None => capra::HostConfig::default(),
    };

    h.host = host;
    h.port = port.into();

    config.host = Some(h);

    match config::save_config(&config) {
        Ok(_) => println!("Config saved"),
        Err(e) => println!("Error saving config: {:?}", e),
    }
}

#[tauri::command]
fn get_state() -> config::ConnectionConfig {
    let config = config::get_config();

    let hostconfig = match config.host {
        Some(h) => h,
        None => capra::HostConfig::default(),
    };

    let host = hostconfig.host;
    let port = hostconfig.port.try_into().unwrap_or(9090);
    let name = "Markhor";

    config::ConnectionConfig {
        host: Mutex::new(Some(host.to_string())),
        port: Mutex::new(Some(port)),
        robot_name: Mutex::new(Some(name.to_string())),
    }
}

fn main() {
    let app;
    {
        // let connection = capra_tauri_ui::establish_connection();
        // configure_database(&connection);
        // let state = fetch_connection(&connection);

        // if state.host.lock().unwrap().is_none() {
        //     println!("No connection found, opening config window");
        //     insert_connection(&connection, "jetson", 8080, "Markhor");
        // }

        app = tauri::Builder::default();
    }

    app.invoke_handler(tauri::generate_handler![set_state, get_state])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
