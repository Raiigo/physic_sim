use bevy::{
    prelude::{
        shape::{Cube, Plane, self},
        App, Assets, Camera, Camera2dBundle, Camera3dBundle, Color, Commands, Component, Handle,
        Mesh, PbrBundle, PointLight, PointLightBundle, Query, Res, ResMut, StandardMaterial,
        Startup, Transform, Update, Vec3, With, Without, MaterialMeshBundle,
    },
    sprite::ColorMaterial,
    time::Time,
    utils::default,
    DefaultPlugins, render::render_resource::PrimitiveTopology,
};
use num::traits::Pow;
use std::{f32::consts::PI, sync::Mutex};

const ROTATE_ANGLE: f32 = PI / 24.0;
const PHI: f32 = PI / 4.0;
const G: f32 = 9.81;
const K: f32 = 2.0;
const M: f32 = 0.2;
const V0: f32 = 10.0;
const ETA: f32 = 0.05;

#[derive(Component)]
struct Cube2;

#[derive(Component)]
struct Line;

#[derive(Component)]
struct Physics {
    mass: f32,
    pos: Vec3,
    speed: Vec3,
    acc: Vec3,
    forces: Vec<Box<dyn Force + Send + Sync>>,
    last_update: f32,
    init: bool,
}

impl Physics {
    fn update(&mut self, delta: f32) {
        self.acc = Vec3::ZERO;
        for f in self.forces.iter() {
            self.acc += f.compute(&self) / self.mass;
        }

        self.speed += self.acc * delta;
        self.pos += self.speed * delta;
    }
}

trait Force {
    fn compute(&self, physics_prop: &Physics) -> Vec3;
}

struct Gravity {
    g: Vec3,
}

impl Force for Gravity {
    fn compute(&self, physics_prop: &Physics) -> Vec3 {
        physics_prop.mass * self.g
    }
}

struct Ressort {
    k: f32,
    pos: Vec3,
}

impl Force for Ressort {
    fn compute(&self, physics_prop: &Physics) -> Vec3 {
        self.k * (self.pos - physics_prop.pos)
    }
}

struct ResistanceAir {
    eta: f32,
}

impl Force for ResistanceAir {
    fn compute(&self, physics_prop: &Physics) -> Vec3 {
        - physics_prop.speed * self.eta
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, update)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut time: ResMut<Time>,
) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(5.0, 0.0, 0.0).looking_at(Vec3 { x: 5.0, y: 0.0, z: 1.0}, Vec3::Y),
        ..default()
    });
    commands.spawn((MaterialMeshBundle {
        mesh: meshes.add(Mesh::from(LineStrip {
            points: vec![
                Vec3::new(5.0, 0.0, 10.0),
                Vec3::new(1.0, 0.0, 0.0),
            ],
        })),
        transform: Transform::from_xyz(0.5, 0.0, 0.0),
        material: materials.add(StandardMaterial::from(Color::BLACK)),
        ..default()
    }, Line));
    commands.spawn(
        (PbrBundle {
            mesh: meshes.add(Mesh::from(Cube { size: 1.0 })),
            material: materials.add(StandardMaterial::from(Color::RED)),
            transform: Transform::from_xyz(0.0, 0.0, 10.0),
            ..default()
        }),
    );
    commands.spawn(PointLightBundle {
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    });
    let gravity = Gravity {
        g: Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        },
    };
    let ressort = Ressort {
        k: K,
        pos: Vec3 { x: 5.0, y: 0.0, z: 10.0 }
    };
    let air = ResistanceAir {
        eta: ETA,
    };
    let m = Mutex::new(5);
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(Cube { size: 1.0 })),
            material: materials.add(StandardMaterial::from(Color::BLUE)),
            transform: Transform::from_xyz(5.0, 0.0, 10.0),
            ..default()
        },
        Physics {
            pos: Vec3 {
                x: 5.0,
                y: 0.0,
                z: 10.0,
            },
            speed: Vec3 {
                x: 5.0,
                y: -10.0,
                z: 0.0
            },
            acc: Vec3::ZERO,
            mass: 0.2,
            forces: vec![Box::new(gravity), Box::new(ressort), Box::new(air)], // Liste des forces appliquées à l'objet, doivent implémenter le trait Force
            last_update: time.elapsed_seconds(),
            init: false,
        },
        Cube2,
    ));
    println!("{}", time.elapsed_seconds());
}

fn update(
    mut camera: Query<&mut Transform, With<Camera>>,
    mut cube: Query<&mut Transform, (Without<Camera>, Without<Cube2>, Without<Line>)>,
    mut cube2: Query<(&mut Transform, &mut Physics), (Without<Camera>, With<Cube2>)>,
    mut line: Query<&mut Handle<Mesh>, With<Line>>,
    mut time: ResMut<Time>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for mut t in camera.iter_mut() {
        // t.rotate_y(ROTATE_ANGLE);
        // println!("{:?}", &t);
    }
    let delta = time.delta_seconds();
    time.update();
    println!("{}, {}", time.elapsed_seconds(), time.delta_seconds());
    for mut c in cube.iter_mut() {
        c.translation.y = x2(time.elapsed_seconds());
        // println!("{}", c.translation);
    }
    for c2 in cube2.iter_mut() {
        let (mut c2_transform, mut c2_physics) = c2;
        if !c2_physics.init {
            c2_physics.last_update = time.elapsed_seconds();
            c2_physics.init = true;
        }
        let d = time.elapsed_seconds() - c2_physics.last_update;
        c2_physics.last_update = time.elapsed_seconds();
        c2_physics.update(d);
        c2_transform.translation = c2_physics.pos;
        // println!("{}", c2_transform.translation);
        println!("{}, {}, {}", c2_physics.acc, c2_physics.speed, c2_physics.pos);
        for mut l in line.iter_mut() {
            let offset = Vec3 { x: 0.5, y: 0.0, z: 0.5 };
            *l = meshes.add(Mesh::from(LineStrip {
                points: vec![
                    Vec3::new(5.0, 0.0, 10.0) - offset,
                    c2_physics.pos - offset,
                ],
            }));
        }
    }
}

// Oscillateur harmonique
fn x(t: f32) -> f32 {
    (M * G) / (K * PHI.cos()) * ((K / M).sqrt() * t + PHI).cos() - ((M * G) / K)
}

// Oscillateur harmonique amorti
fn x2(t: f32) -> f32 {
    let delta: f32 = (ETA / M).pow(2) - 4.0 * (K / M);
    // println!("{}", delta);
    if delta > 0.0 {
        let l1 = (-(ETA / M) - delta.sqrt()) / (2.0);
        let l2 = (-(ETA / M) + delta.sqrt()) / (2.0);
        let A = -V0 / (l2 - l1);
        let B = V0 / (l1 - l2);
        return A * (l1 * t).exp() + B * (l2 * t).exp() - ((M * G) / K);
    } else if delta < 0.0 {
        let u = -(ETA / M) / 2.0;
        let v = (-delta).sqrt() / 2.0;
        return (V0 / v.abs()) * (u * t).exp() * (v.abs() * t).sin() - ((M * G) / K);
    }
    0.0
}

#[derive(Debug, Clone)]
pub struct LineStrip {
    pub points: Vec<Vec3>,
}

impl From<LineStrip> for Mesh {
    fn from(line: LineStrip) -> Self {
        // This tells wgpu that the positions are a list of points
        // where a line will be drawn between each consecutive point
        let mut mesh = Mesh::new(PrimitiveTopology::LineStrip);

        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, line.points);
        mesh
    }
}