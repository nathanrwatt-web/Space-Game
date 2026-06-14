// Persistence layer: knows about the on-disk world folders, NOT about game data.
// Layout: Worlds/<name>/system.ron (+ meta.ron for the start-screen list).
// Future subsystems (economy, thrust, ...) persist their own file in the same folder
// via write_ron/read_ron — nothing here changes.

use bevy::prelude::*;
use serde::{Serialize, Deserialize, de::DeserializeOwned};
use std::fs;
use std::path::{Path, PathBuf};

pub const WORLDS_ROOT: &str = "Worlds";
pub const SYSTEM_FILE: &str = "system.ron";
pub const META_FILE: &str = "meta.ron";

// the world currently loaded / being saved to (None on the start screen)
#[derive(Resource, Default)]
pub struct CurrentWorld(pub Option<String>);

// small display info, stored next to the world data and shown on the start screen
#[derive(Serialize, Deserialize, Clone, Default)]
pub struct WorldMeta {
    pub sim_time: f64,
}

pub struct WorldSlot {
    pub name: String,
    pub meta: WorldMeta,
}

pub fn world_dir(name: &str) -> PathBuf {
    Path::new(WORLDS_ROOT).join(name)
}
pub fn system_path(name: &str) -> PathBuf {
    world_dir(name).join(SYSTEM_FILE)
}
pub fn meta_path(name: &str) -> PathBuf {
    world_dir(name).join(META_FILE)
}

// generic RON I/O — creates parent dirs on write
pub fn write_ron<T: Serialize>(path: &Path, value: &T) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let text = ron::ser::to_string_pretty(value, ron::ser::PrettyConfig::default())
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    fs::write(path, text)
}

pub fn read_ron<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    ron::from_str::<T>(&text).map_err(|e| e.to_string())
}

// every world folder under Worlds/, sorted by name
pub fn list_worlds() -> Vec<WorldSlot> {
    let mut slots = Vec::new();
    let Ok(entries) = fs::read_dir(WORLDS_ROOT) else { return slots; };
    for entry in entries.flatten() {
        if !entry.path().is_dir() {
            continue;
        }
        let Some(name) = entry.file_name().to_str().map(str::to_string) else { continue; };
        let meta = read_ron::<WorldMeta>(&meta_path(&name)).unwrap_or_default();
        slots.push(WorldSlot { name, meta });
    }
    slots.sort_by(|a, b| a.name.cmp(&b.name));
    slots
}

// first free "world_N" folder name
pub fn next_world_name() -> String {
    let mut n = 1;
    loop {
        let name = format!("world_{n}");
        if !world_dir(&name).exists() {
            return name;
        }
        n += 1;
    }
}
