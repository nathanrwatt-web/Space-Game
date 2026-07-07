// handles worlds on disk in the Worlds/name/*.ron files
use bevy::prelude::*;
use serde::{Deserialize, Serialize, de::DeserializeOwned};
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
        fs::create_dir_all(parent)?; // create directory if not already there 
    }
    let text = ron::ser::to_string_pretty(value, ron::ser::PrettyConfig::default())
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    fs::write(path, text)
}

pub fn read_ron<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    ron::from_str::<T>(&text).map_err(|e| e.to_string())
}

// generic: every subdirectory name under `root` sorted
pub fn list_dirs(root: &str) -> Vec<String> {
    let mut names = Vec::new();
    let Ok(entries) = fs::read_dir(root) else {
        return names;
    };
    for entry in entries.flatten() {
        if !entry.path().is_dir() {
            continue;
        }
        if let Some(name) = entry.file_name().to_str() {
            names.push(name.to_string());
        }
    }
    names.sort();
    names
}

// every world folder under Worlds/ + meta data
pub fn list_worlds() -> Vec<WorldSlot> {
    list_dirs(WORLDS_ROOT)
        .into_iter()
        .map(|name| {
            let meta = read_ron::<WorldMeta>(&meta_path(&name)).unwrap_or_default();
            WorldSlot { name, meta }
        })
        .collect()
}

// first free # for world_# naming
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
