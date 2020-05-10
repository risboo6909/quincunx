use std::collections::HashMap;

use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::nalgebra as na;

use ncollide3d::shape::{Ball, ShapeHandle};

use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{DefaultBodySet, ColliderDesc, DefaultColliderSet, RigidBodyDesc, BodyPartHandle, DefaultColliderHandle};

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

        let g: na::Vector3<N> = na::Vector3::new(N::zero(), N::from_f32(-9.81).unwrap(), N::zero());

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

    pub fn make_sphere(&mut self, window: &mut Window, x: N, y: N, z: N, radius: f32) -> SceneNode {

        let node = window.add_sphere(radius);
    
        // build the rigid body
        let rb = RigidBodyDesc::new().translation(na::Vector3::new(x, y, z)).build();
        let rb_handle = self.bodies.insert(rb);
    
        // build the collider
        let uid = self.get_uid();
        let co = ColliderDesc::new(ShapeHandle::new(Ball::new(N::from_f32(radius).unwrap())));
        let col_handle = self.colliders.insert(co.density(N::one()).build(BodyPartHandle(rb_handle, uid))); 
    
        self.objects.insert(col_handle, node.clone());

        node
    }

    pub fn get_uid(&mut self) -> usize {
        self.uid += 1;
        self.uid
    }

}
