use kiss3d::scene::SceneNode;
use kiss3d::light::Light;
use kiss3d::window::{State, Window};
use kiss3d::event::{Action, WindowEvent};
use kiss3d::camera::{Camera, ArcBall};
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::renderer::Renderer;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::nalgebra as na;

use ncollide3d::shape::{Ball, ShapeHandle};
use nphysics3d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};
use nphysics3d::object::{DefaultBodySet, ColliderDesc, DefaultColliderSet, RigidBodyDesc, BodyPartHandle, DefaultBodyHandle,
DefaultColliderHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;

pub struct World<N: na::RealField> {
    mworld: DefaultMechanicalWorld<N>,
    gworld: DefaultGeometricalWorld<N>,
    /// The set of physics bodies
    pub bodies: DefaultBodySet<N>,
    /// The set of colliders, each attached to a body-part
    pub colliders: DefaultColliderSet<N>,
    /// Joint constraints on bodies
    pub joint_constraints: DefaultJointConstraintSet<N>,
    /// Forces acting on bodies
    pub force_generators: DefaultForceGeneratorSet<N>,
    /// Simulation time point
    pub time: N,
}

struct AppState {
    cam: ArcBall,
    c: SceneNode,

    rb_handler: DefaultBodyHandle,
    col_handler: DefaultColliderHandle,
    world: World<f32>,

    //rot: UnitQuaternion<f32>,
}

impl State for AppState {

    fn cameras_and_effect_and_renderer(&mut self) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn Renderer>,
        Option<&mut dyn PostProcessingEffect>,
    )  {
        (Some(&mut self.cam), None, None, None)
    }

    fn step(&mut self, window: &mut Window) {

        let pos = self.world.colliders.get(self.col_handler).unwrap().position();
        self.c.set_local_transformation(*pos);

        // self.c.prepend_to_local_translation(&na::Translation3::new(0.0, -0.001, 0.0));

        self.world.mworld.step(
            &mut self.world.gworld, 
            &mut self.world.bodies, 
            &mut self.world.colliders, 
            &mut self.world.joint_constraints, 
            &mut self.world.force_generators
        );
    }
}

fn main() {
    
    /*
     * World
     */
    let mut mworld = DefaultMechanicalWorld::new(na::Vector3::new(0.0, -9.81, 0.0));
    let mut gworld = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let mut joint_constraints = DefaultJointConstraintSet::new();
    let mut force_generators = DefaultForceGeneratorSet::new();

    let eye = na::Point3::new(1f32, 1.0, 1.0);
    let at = na::Point3::origin();

    let mut window = Window::new("Quincunx");

    // camera
    let arc_ball = ArcBall::new(eye, at);
    
    // sphere
    let mut c = window.add_sphere(0.1);
    
    // build the rigid body
    let rb = RigidBodyDesc::new().translation(na::Vector3::new(0.0, 0.0, 0.0)).build();
    let rb_handle = bodies.insert(rb);

    // build the collider
    let co = ColliderDesc::new(ShapeHandle::new(Ball::new(0.1)));
    let col_handle = colliders.insert(co.density(1.0).build(BodyPartHandle(rb_handle, 0)));

    c.set_color(1.0, 0.0, 0.0);
    c.set_local_translation(na::Translation3::new(0.0, 0.0, 0.0));

    window.set_light(Light::StickToCamera);

    mworld.step(&mut gworld, &mut bodies, &mut colliders, &mut joint_constraints, &mut force_generators);

    let state = AppState { c, cam: arc_ball, 
        rb_handler: rb_handle,
        col_handler: col_handle,
        world: World{
        mworld,
        gworld,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
        time: 0.0,
    } };

    window.render_loop(state)
}
