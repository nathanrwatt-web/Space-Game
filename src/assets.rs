// Central asset manifest + registry.
//
// Loaded once at boot to reduce loading mid run 

use std::collections::HashMap;

use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use bevy::scene::SceneInstanceReady;

// Model files preloaded as GLTF scenes at startup, relative to the `assets/` folder.
pub const PRELOAD_SCENES: &[&str] = &["Planet.glb"];

// Registry of preloaded scene handles, keyed by asset path.
#[derive(Resource, Default)]
pub struct GameAssets {
    scenes: HashMap<String, Handle<Scene>>,
}

impl GameAssets {
    // The preloaded scene handle for `path`, if it was listed in `PRELOAD_SCENES`.
    pub fn scene(&self, path: &str) -> Option<Handle<Scene>> {
        self.scenes.get(path).cloned()
    }
}

// Placed on a `SceneRoot` entity to remove GLTF nodes once the scene finishes spawning.
// Any node whose `Name` contains one of these strings is despawned
#[derive(Component, Clone, Default)]
pub struct HideSceneNodes(pub Vec<String>);

// Placed on a `SceneRoot` entity to switch matched GLTF nodes to additive blending once
// the scene spawns. Use for grayscale-on-black overlay layers with no alpha channel —
// e.g. Planet.glb's `planet cloud` (white clouds on black): additive makes the black
// read as transparent so the surface shows through, and the white clouds glow on top.
#[derive(Component, Clone, Default)]
pub struct AdditiveNodes(pub Vec<String>);

pub struct AssetsPlugin;

impl Plugin for AssetsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GameAssets>()
            .add_systems(PreStartup, preload_scenes)
            .add_observer(strip_hidden_nodes)
            .add_observer(set_additive_nodes);
    }
}

// Kick off every manifest load at boot so the glb files are warm by spawn time
fn preload_scenes(asset_server: Res<AssetServer>, mut assets: ResMut<GameAssets>) {
    for path in PRELOAD_SCENES {
        let handle = asset_server.load(GltfAssetLabel::Scene(0).from_asset(path.to_string()));
        assets.scenes.insert(path.to_string(), handle);
    }
}

// Once a scene instance is ready, despawn any nodes the body asked to hide
// Runs for every scene but no-ops unless the root carries a non-empty HideSceneNodes.
fn strip_hidden_nodes(
    ready: On<SceneInstanceReady>,
    roots: Query<&HideSceneNodes>,
    children: Query<&Children>,
    names: Query<&Name>,
    mut commands: Commands,
) {
    let root = ready.entity;
    let Ok(hide) = roots.get(root) else {
        return;
    };
    if hide.0.is_empty() {
        return;
    }
    for descendant in children.iter_descendants(root) {
        if let Ok(name) = names.get(descendant)
            && hide.0.iter().any(|h| name.as_str().contains(h.as_str()))
        {
            commands.entity(descendant).despawn();
        }
    }
}

// switch materials of nodes in AdditiveNodes when the scene is ready
fn set_additive_nodes(
    ready: On<SceneInstanceReady>,
    roots: Query<&AdditiveNodes>,
    children: Query<&Children>,
    names: Query<&Name>,
    mesh_materials: Query<&MeshMaterial3d<StandardMaterial>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let root = ready.entity;
    let Ok(additive) = roots.get(root) else {
        return;
    };
    if additive.0.is_empty() {
        return;
    }
    for node in children.iter_descendants(root) {
        let Ok(name) = names.get(node) else {
            continue;
        };
        if !additive.0.iter().any(|h| name.as_str().contains(h.as_str())) {
            continue;
        }
        for entity in std::iter::once(node).chain(children.iter_descendants(node)) {
            if let Ok(mesh_material) = mesh_materials.get(entity)
                && let Some(material) = materials.get_mut(&mesh_material.0)
            {
                material.alpha_mode = AlphaMode::Add;
            }
        }
    }
}
