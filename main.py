from random import random
from turtle import Vec2D
from Box2D.examples.framework import Framework, Keys, main
from Box2D import b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape, b2_pi, b2Random, b2Vec2, b2_epsilon
from math import sqrt

def create_car(
            world, offset, wheel_radius, wheel_separation, density=1.0, wheel_friction=0.9,
            scale=(1.0, 1.0), chassis_vertices=None, wheel_axis=(0.0, 1.0), wheel_torques=[30.0, 20.0],
            wheel_drives=[True, False], hz=4.0, zeta=0.7, **kwargs):
            
            x_offset, y_offset = offset
            scale_x, scale_y = scale
            if chassis_vertices is None:
                chassis_vertices=[
                    (-1.5, -0.5),
                    (1.5, -0.5),
                    (1.5, 0.0),
                    (0.0, 0.9),
                    (-1.15, 0.9),
                    (-1.5, 0.2),
                ]

            chassis_vertices = [(scale_x * x, scale_y * y) for x, y, in chassis_vertices]
            radius_scale = sqrt(scale_x **2 + scale_y **2)
            wheel_radius *= radius_scale

            chassis = world.CreateDynamicBody(
                position = (x_offset, y_offset),
                fixtures = b2FixtureDef(
                    shape = b2PolygonShape(vertices = chassis_vertices),
                    density=density,
                )
            )

            wheels, springs = [], []
            wheel_xs = [-wheel_separation * scale_x / 2.0, wheel_separation * scale_x / 2.0]

            for x, torque, drive in zip(wheel_xs, wheel_torques, wheel_drives):
                wheel = world.CreateDynamicBody(
                    position=(x_offset + x, y_offset - wheel_radius),
                    fixtures = b2FixtureDef(
                        shape = b2CircleShape(radius=wheel_radius),
                        density=density,
                    )
                )

                spring = world.CreateWheelJoint(
                    bodyA=chassis,
                    bodyB=wheel,
                    anchor=wheel.position,
                    axis=wheel_axis,
                    motorSpeed = 0.0,
                    maxMotorTorque=torque,
                    enableMotor=drive,
                    frequencyHz=hz,
                    dampingRatio=zeta
                )

                wheels.append(wheel)
                springs.append(spring)
            
            return chassis, wheels, springs

class ArresterBed (Framework):
    name = "Arrester Bed simulation SYPT"
    description = "c-create grain, d-fwd, a-bwd, e-hzdown, q-hzup, v-changeviewpoint"

    hz = 900000.0
    zeta = 0.7
    speed = 50

    def __init__(self) -> None:
        super(ArresterBed, self).__init__()

        # initialize ground and bed
        ground = self.world.CreateStaticBody(
            shapes = b2EdgeShape(vertices=[(-80, 0), (20, 0)])
        )
        bed = self.world.CreateStaticBody(
            shapes = b2EdgeShape(vertices = [(20.0, -5.0), (60.0, -5.0)])
        )
        bed2 = self.world.CreateStaticBody(
            shapes = b2EdgeShape(vertices = [(20.0, 0.0), (20.0, -5.0)])
        )
        bed3 = self.world.CreateStaticBody(
            shapes = b2EdgeShape(vertices = [(60.0, 0.0), (60.0, -5.0)])
        )
        platform = self.world.CreateStaticBody(
            shapes = b2EdgeShape(vertices = [(-100, -40), (100, -40)])
        )

        #initialize car
        car, wheels, springs = create_car(self.world, offset=(-40.0, 1.0), wheel_radius=0.4, wheel_separation=2.0, scale=(1, 1))
        self.car = car
        self.wheels = wheels
        self.springs = springs

        #initialize the bodies - CHANGE RADIUS HERE
        self.radius = radius = 0.09
        column_count = 200
        row_count = 60

        for j in range(column_count):
            for i in range(row_count):
                self.CreateCircle(((2.1 * j + 1 + 0.01 * i) * radius,
                                  (2 * i + 1) * radius))

        self.world.gravity = (0, -9.8)

    def CreateCircle(self, pos):
        #CHANGE DENSITY, FRICTION, RESTITUTION HERE
        fixture = b2FixtureDef(shape = b2CircleShape(radius = self.radius, pos=(30.0, -4.0)), density = 1, friction = 0.55,  restitution = 0.41)
        self.world.CreateDynamicBody(position = pos, fixtures = fixture)

    def Keyboard(self, key):
        if key == Keys.K_c:
            for i in range(500):
                self.CreateCircle((2.0 * random() + 20.0, self.radius * (1.0 + random())))
        if key == Keys.K_a:
            self.springs[0].motorSpeed += 1
        
        elif key == Keys.K_s:
            self.springs[0].motorSpeed = 0

        elif key == Keys.K_d:
            self.springs[0].motorSpeed -= 1

        elif key in (Keys.K_q, Keys.K_e):
            if key == Keys.K_q:
                self.hz = max(0, self.hz - 1.0)
            
            else: 
                self.hz += 1.0

            for spring in self.springs:
                spring.springFrequencyHz = self.hz

        elif key == Keys.K_v:
            self.viewCenter = (self.car.position.x, 0)

        elif key == Keys.K_b:
            self.viewCenter = (30, 0)

        elif key == Keys.K_p:
            #Launch velocity
            self.car.linearVelocity = (30, 0)
            self.car.angularVelocity = 0


    def Step(self, settings):
            super(ArresterBed, self).Step(settings)
            self.Print("frequency = %g hz, damping ratio = %g" % (self.hz, self.zeta))
            

if __name__ == "__main__":
    main(ArresterBed)
