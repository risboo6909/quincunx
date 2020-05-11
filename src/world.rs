use rand::Rng;
use std::collections::HashMap;

use kiss3d::nalgebra as na;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

use ncollide3d::math::Vector;
use ncollide3d::pipeline::object::CollisionGroups;
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};

use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderHandle,
    DefaultColliderSet, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

const PINS_INIT_OFFSET_Y: f32 = 2.1;
const BALL_INIT_OFFST_Y: f32 = 6f32;
const BOARD_HEIGHT: f32 = 15f32;
const BOARD_WIDTH: f32 = 6f32;
const PIN_SIDE: f32 = 0.16;
const PIN_HEIGHT: f32 = 0.12;

const BALLS_COUNT: usize = 500;
const BALL_RADIUS: f32 = 0.07;

const PARTITION_HEIGHT: f32 = 4f32;

pub struct World<N: na::RealField> {
    pub mworld: DefaultMechanicalWorld<N>,
    pub gworld: DefaultGeometricalWorld<N>,

    /// The set of physics bodies
    pub bodies: DefaultBodySet<N>,

    /// The set of colliders, each attached to a body-part
    pub colliders: DefaultColliderSet<N>,

    /// Joint constraints on bodies
    pub joint_constraints: DefaultJointConstraintSet<N>,

    /// Forces acting on bodies
    pub force_generators: DefaultForceGeneratorSet<N>,

    pub objects: HashMap<DefaultColliderHandle, SceneNode>,

    pub uid: usize,
}

impl<'a, N: na::RealField> World<N> {
    pub fn new() -> Self {
        let objects = HashMap::new();
        let g: na::Vector3<N> = na::Vector3::new(N::zero(), N::zero(), N::from_f32(9.81).unwrap());

        let mworld = DefaultMechanicalWorld::new(g);
        let gworld = DefaultGeometricalWorld::new();
        let bodies = DefaultBodySet::new();
        let colliders = DefaultColliderSet::new();
        let joint_constraints = DefaultJointConstraintSet::new();
        let force_generators = DefaultForceGeneratorSet::new();

        World {
            mworld,
            gworld,
            bodies,
            colliders,
            joint_constraints,
            force_generators,
            objects,
            uid: 0,
        }
    }

    fn make_sphere(
        &mut self,
        window: &mut Window,
        x: f32,
        y: f32,
        z: f32,
        radius: f32,
    ) -> SceneNode {
        let node = window.add_sphere(radius);

        let rb = RigidBodyDesc::new()
            .status(BodyStatus::Dynamic)
            .translation(na::Vector3::new(
                N::from_f32(x).unwrap(),
                N::from_f32(y).unwrap(),
                N::from_f32(z).unwrap(),
            ))
            .kinematic_translations(Vector::new(false, true, false))
            .mass(na::convert(5.0))
            .build();
        let rb_handle = self.bodies.insert(rb);

        let co = ColliderDesc::new(ShapeHandle::new(Ball::new(N::from_f32(radius).unwrap())));

        let col_handle = self.colliders.insert(
            co.margin(na::convert(0.0001))
                .density(na::one())
                .collision_groups(CollisionGroups::new().with_membership(&[1]))
                .build(BodyPartHandle(rb_handle, 0)),
        );

        self.objects.insert(col_handle, node.clone());

        node
    }

    fn make_box(
        &mut self,
        window: &mut Window,
        x: f32,
        y: f32,
        z: f32,
        wx: f32,
        wy: f32,
        wz: f32,
        rot: f32,
    ) -> SceneNode {
        let node = window.add_cube(wx, wy, wz);

        let rb = RigidBodyDesc::new()
            .gravity_enabled(false)
            .translation(na::Vector3::new(
                N::from_f32(x).unwrap(),
                N::from_f32(y).unwrap(),
                N::from_f32(z).unwrap(),
            ))
            .status(BodyStatus::Static)
            .rotation(na::Vector3::new(
                na::zero(),
                N::from_f32(rot).unwrap(),
                na::zero(),
            ))
            .build();

        let rb_handle = self.bodies.insert(rb);

        let half_wx = N::from_f32(0.5 * wx).unwrap();
        let half_wy = N::from_f32(0.5 * wy).unwrap();
        let half_wz = N::from_f32(0.5 * wz).unwrap();

        let co = ColliderDesc::new(ShapeHandle::new(Cuboid::new(na::Vector3::new(
            half_wx, half_wy, half_wz,
        ))));

        let col_handle = self.colliders.insert(
            co.density(na::convert(5.0))
                .collision_groups(
                    CollisionGroups::new()
                        .with_membership(&[0])
                        .with_whitelist(&[1]),
                )
                .build(BodyPartHandle(rb_handle, 0)),
        );

        self.objects.insert(col_handle, node.clone());

        node
    }

    fn make_pin(&mut self, window: &mut Window, x: f32, y: f32, z: f32, wx: f32, wy: f32, wz: f32) {
        let node = window.add_cube(na::convert(wx), na::convert(wy), na::convert(wz));

        let rb = RigidBodyDesc::new()
            .gravity_enabled(false)
            .translation(na::Vector3::new(
                N::from_f32(x).unwrap(),
                N::from_f32(y).unwrap(),
                N::from_f32(z).unwrap(),
            ))
            .status(BodyStatus::Static)
            .rotation(na::Vector3::new(na::zero(), na::convert(69.9), na::zero()))
            .build();

        let rb_handle = self.bodies.insert(rb);

        let half_wx = N::from_f32(0.5 * wx).unwrap();
        let half_wy = N::from_f32(0.5 * wy).unwrap();
        let half_wz = N::from_f32(0.5 * wz).unwrap();

        let co = ColliderDesc::new(ShapeHandle::new(Cuboid::new(na::Vector3::new(
            half_wx, half_wy, half_wz,
        ))));

        let col_handle = self.colliders.insert(
            co.density(na::convert(5.0))
                .margin(na::convert(0.0001))
                .collision_groups(
                    CollisionGroups::new()
                        .with_membership(&[0])
                        .with_whitelist(&[1]),
                )
                .build(BodyPartHandle(rb_handle, 0)),
        );

        self.objects.insert(col_handle, node.clone());
    }

    pub fn make_board(&mut self, window: &mut Window, x: f32, y: f32, z: f32) {
        let mut rng = rand::thread_rng();

        for _ in 0..BALLS_COUNT {
            let mut sphere = self.make_sphere(
                window,
                1.0 - 2.0 * rng.gen::<f32>(),
                2.0 * BALL_RADIUS,
                -BALL_INIT_OFFST_Y + -2.0 * rng.gen::<f32>(),
                BALL_RADIUS,
            );

            sphere.set_color(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>());
        }

        let mut boxx = self.make_box(window, x, y, z, BOARD_WIDTH, 0.1, BOARD_HEIGHT, 0.0);
        boxx.set_color(0.0, 0.7, 0.0);

        let mut border1 = self.make_box(
            window,
            x - 0.5 * BOARD_WIDTH,
            y + 0.2,
            z - 0.5 * BOARD_HEIGHT,
            0.2,
            0.6,
            9.7,
            10.03,
        );
        border1.set_color(0.0, 0.0, 1.0);

        let mut border2 = self.make_box(
            window,
            x + 0.5 * BOARD_WIDTH,
            y + 0.2,
            z - 0.5 * BOARD_HEIGHT,
            0.2,
            0.6,
            9.7,
            -10.03,
        );
        border2.set_color(0.0, 0.0, 1.0);

        let mut floor = self.make_box(
            window,
            x,
            y + 0.2,
            z + 0.5 * BOARD_HEIGHT,
            BOARD_WIDTH,
            0.6,
            0.2,
            0.0,
        );
        floor.set_color(0.0, 0.0, 1.0);

        let mut left = self.make_box(
            window,
            x - 0.5 * BOARD_WIDTH,
            y + 0.2,
            z,
            0.07,
            0.6,
            BOARD_HEIGHT,
            0.0,
        );
        left.set_color(0.0, 0.0, 1.0);

        let mut right = self.make_box(
            window,
            x + 0.5 * BOARD_WIDTH,
            y + 0.2,
            z,
            0.07,
            0.6,
            BOARD_HEIGHT,
            0.0,
        );
        right.set_color(0.0, 0.0, 1.0);

        for n in 0..13 {
            let mut part = self.make_box(
                window,
                x - 0.5 * BOARD_WIDTH + 0.5 * n as f32,
                y + 0.2,
                z + 0.5 * BOARD_HEIGHT - 0.5 * PARTITION_HEIGHT,
                0.04,
                0.6,
                PARTITION_HEIGHT,
                0.0,
            );
            part.set_color(0.0, 0.0, 1.0);
        }

        for row in 1..13 {

            for col in 0..=(14 - 1 * ((row + 1) % 2)) {
            
                let offset_x = if row % 2 == 0 { -2.45 } else { -2.65 };

                self.make_pin(
                    window,
                    x as f32 + offset_x + 0.38 * col as f32,
                    y as f32 + PIN_HEIGHT,
                    z as f32 - PINS_INIT_OFFSET_Y + 0.38 * row as f32,
                    PIN_SIDE,
                    PIN_SIDE,
                    PIN_SIDE,
                );

            }

        }
        
    }
}
