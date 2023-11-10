# SPDX-License-Identifier: BSD-3-Clause

# flake8: noqa F401

import numpy as np
import random

from vendeeglobe import (
    Checkpoint,
    Heading,
    Instructions,
    Location,
    MapProxy,
    Vector,
    WeatherForecast,
    config,
)
from vendeeglobe.utils import distance_on_surface, goto

CREATOR = "Mads"  # This is your team name

class plan:
    def __init__(self, course, goal_index, goal=None, radius=None):

        self.goal_index = goal_index
        self.course = course

        if goal is None:
            self.goal = np.array([course[-1].latitude, course[-1].longitude])
        else:
            self.goal = goal

        if radius is None:
            self.radius = course[-1].radius
        else:
            self.radius = radius

        self.done = False

    def go(self, longitude, latitude, dt, speed):

        for ch in self.course:
            if ch.reached == True:
                continue

            dist = distance_on_surface(
                longitude1=longitude,
                latitude1=latitude,
                longitude2=ch.longitude,
                latitude2=ch.latitude,
            )

            if dist < ch.radius:
                ch.reached = True

        for ch in self.course:
            if ch.reached == True:
                continue

            jump = dt * np.linalg.norm(speed)
            if dist < 2.0 * ch.radius + jump:
                sail = min(ch.radius / jump, 1)
            else:
                sail = 1.0
            if dist < ch.radius:
                ch.reached = True
            if not ch.reached:
                location = Location(
                    longitude=ch.longitude, latitude=ch.latitude
                )
                break

        final_ch = self.course[-1]
        if final_ch.reached:
            location = Location(
                longitude=final_ch.longitude, latitude=final_ch.latitude
            )
            sail = 1.0
            return location, sail, True

        return location, sail, False


class Bot:
    """
    This is the ship-controlling bot that will be instantiated for the competition.
    """

    def __init__(self):
        self.team = CREATOR  # Mandatory attribute
        self.avatar = 1  # Optional attribute

        self.last_location = 0
        self.plan = None
        self.checkpoint_remaining = [1, 2]



        self.checkpoint_1_lat_long = np.array([2.806318, -168.943864])
        self.checkpoint_2_lat_long = np.array([-15.668984, 77.674694])

        self.last_lattitude = None
        self.last_longitude = None

        # Testing
        #self.last_location = 2
        #self.checkpoint_remaining = [1, 2]

        # 0 start, 1 C1, 2 C2

        # start route map
        self.routes = {}
        self.routes[0] = {}
        self.routes[1] = {}
        self.routes[2] = {}

        course_start_to_C1 = [
            Checkpoint(latitude=43.797109, longitude=-11.264905, radius=50),
            Checkpoint(longitude=-29.908577, latitude=17.999811, radius=50),
            Checkpoint(latitude=-11.441808, longitude=-29.660252, radius=50),
            Checkpoint(longitude=-63.240264, latitude=-61.025125, radius=50),
            Checkpoint(latitude=2.806318, longitude=-168.943864, radius=1990.0), # 2000
            ]

        course_start_to_C1_through_panama = [
            Checkpoint(39.439980, -29.228266, radius=50),
            Checkpoint(17.877642, -63.931619, radius=10),
            Checkpoint(10.550260, -80.261503, radius=10), # above to prepare
            Checkpoint(9.086190, -79.818728, radius=10), # actual cannal
            Checkpoint(6.138857, -78.953983, radius=30),
            Checkpoint(latitude=2.806318, longitude=-168.943864, radius=1990.0),  # 2000
        ]

        self.routes[0][1] = [plan(course_start_to_C1_through_panama, goal_index=1),
                             #plan(course_start_to_C1, goal_index=1),
                            ]

        course_C1_to_C2 = [
            Checkpoint(latitude=-62.052286, longitude=169.214572, radius=50.0),
            Checkpoint(latitude=-15.668984, longitude=77.674694, radius=1190.0),  # 1200
        ]

        #5.098613, 126.091818

        course_C1_to_C2_quick = [
            Checkpoint(5.098613, 126.091818, radius=50.0),
            Checkpoint(0.737651, 119.503680, radius=50.0),
            Checkpoint(-6.455657, 117.306415, radius=50.0),
            Checkpoint(-4.519411, 108.035830, radius=10.0),
            Checkpoint(-5.847248, 105.918941, radius=30.0), # tiny gap
            Checkpoint(-7.102692, 102.609311, radius=30.0),  # tiny gap
            #Checkpoint(latitude=-15.668984, longitude=77.674694, radius=1190.0),  # 1200
            Checkpoint(-7.050042, 77.820318, radius=20.0),  # 1200
        ]

        self.routes[1][2] = [plan(course_C1_to_C2_quick, goal_index=2),
                             #plan(course_C1_to_C2, goal_index=2),
                             ]

        course_C2_to_start = [
            Checkpoint(latitude=-39.438937, longitude=19.836265, radius=50.0),
            Checkpoint(latitude=14.881699, longitude=-21.024326, radius=50.0),
            Checkpoint(latitude=44.076538, longitude=-18.292936, radius=50.0),
            Checkpoint(
                latitude=config.start.latitude,
                longitude=config.start.longitude,
                radius=5,
            )
            ]

        course_C2_to_start_quick = [

            Checkpoint(14.911203, 55.438251, radius=30.0), # get to entry
            Checkpoint(11.186727, 43.980339, radius=16.0), # first turn in entry
            Checkpoint(25.045441, 36.245365, radius=10.0), # middle of cannal
            Checkpoint(28.397641, 33.300382, radius=10.0), # first bend in narrow
            Checkpoint(29.611130, 32.496083, radius=5.0),  # second bend in narrow
            Checkpoint(33.335618, 31.914460, radius=15.0), # mediteranian
            #Checkpoint(35.683605, 12.347973, radius=15.0), # south of malta
            Checkpoint(36.578350, 13.160854, radius=15.0),  # south of malta
            #Checkpoint(38.891133, 11.153104, radius=15.0),  # north of malta
            Checkpoint(38.241445, 11.189234, radius=15.0),  # north of malta
            Checkpoint(35.984336, -5.232362, radius=15.0),  # out of gibraltar
            Checkpoint(35.849286, -9.661498, radius=15.0),  # out of gibraltar
            Checkpoint(44.141949, -12.566433, radius=50.0), # get through to france

            Checkpoint(
                latitude=config.start.latitude,
                longitude=config.start.longitude,
                radius=5,
            )
        ]

        self.routes[2][0] = [plan(course_C2_to_start_quick, goal_index=0),
                             #plan(course_C2_to_start, goal_index=0),
                             ]

        self.old_course = [
            Checkpoint(latitude=43.797109, longitude=-11.264905, radius=50),
            Checkpoint(longitude=-29.908577, latitude=17.999811, radius=50),
            Checkpoint(latitude=-11.441808, longitude=-29.660252, radius=50),
            Checkpoint(longitude=-63.240264, latitude=-61.025125, radius=50),
            Checkpoint(latitude=2.806318, longitude=-168.943864, radius=1990.0), # 2000
            Checkpoint(latitude=-62.052286, longitude=169.214572, radius=50.0),
            Checkpoint(latitude=-15.668984, longitude=77.674694, radius=1190.0), # 1200
            Checkpoint(latitude=-39.438937, longitude=19.836265, radius=50.0),
            Checkpoint(latitude=14.881699, longitude=-21.024326, radius=50.0),
            Checkpoint(latitude=44.076538, longitude=-18.292936, radius=50.0),
            Checkpoint(
                latitude=config.start.latitude,
                longitude=config.start.longitude,
                radius=5,
            ),
        ]

        self.last_wiggle_sign = 1



    def run(
        self,
        t: float,
        dt: float,
        longitude: float,
        latitude: float,
        heading: float,
        speed: float,
        vector: np.ndarray,
        forecast: WeatherForecast,
        world_map: MapProxy,
    ):
        """
        This is the method that will be called at every time step to get the
        instructions for the ship.

        Parameters
        ----------
        t:
            The current time in hours.
        dt:
            The time step in hours.
        longitude:
            The current longitude of the ship.
        latitude:
            The current latitude of the ship.
        heading:
            The current heading of the ship.
        speed:
            The current speed of the ship.
        vector:
            The current heading of the ship, expressed as a vector.
        forecast:
            The weather forecast for the next 5 days.
        world_map:
            The map of the world: 1 for sea, 0 for land.
        """

        # check if there is a plan
        if self.plan is None:
            # if there is no plan, make one
            if len(self.checkpoint_remaining) == 0:
                # Both checkpoints cleared, go home!
                goal = 0

                possible_routes = self.routes[self.last_location][goal] # crashes

                if len(possible_routes) == 0:
                    # Error
                    raise ValueError("No route ", self.last_location, " to ", goal)

                elif len(possible_routes) == 1:
                    # Get that plan
                    plan = possible_routes[0]  # choose only route
                else:
                    # choose among routes
                    raise ValueError("More than one route from ", self.last_location, " to ", goal)

                    pass

                self.plan = plan
                print("Planning to go for ", goal)

            elif len(self.checkpoint_remaining) == 1:
                # One checkpoint remaining, go there!
                goal = self.checkpoint_remaining[0]

                possible_routes = self.routes[self.last_location][goal]

                if len(possible_routes) == 0:
                    # Error
                    raise ValueError("No route ", self.last_location, " to ", goal)

                elif len(possible_routes) == 1:
                    # Get that plan
                    plan = possible_routes[0]  # choose only route
                else:
                    # choose among routes
                    raise ValueError("More than one route from ", self.last_location, " to ", goal)

                    pass

                self.plan = plan
                print("Planning to go for ", goal)

            else:
                # Need to choose between checkpoints! hard coded to 1 for now
                goal = 1

                possible_routes = self.routes[self.last_location][goal]

                if len(possible_routes) == 0:
                    # Error
                    raise ValueError("No route ", self.last_location, " to ", goal)

                elif len(possible_routes) == 1:
                    # Get that plan
                    plan = possible_routes[0]  # choose only route
                else:
                    # choose among routes
                    raise ValueError("More than one route from ", self.last_location, " to ", goal)

                    pass

                self.plan = plan
                print("Planning to go for ", goal)

        else:
            plan = self.plan


        # Follow the plan
        location, sail, done = plan.go(longitude=longitude, latitude=latitude, dt=dt, speed=speed)

        # Check if we are done
        if done:
            print("Reached ", plan.goal_index)

            print("Checkpoints remaining before logic: ", self.checkpoint_remaining)

            self.last_location = plan.goal_index
            if self.last_location in self.checkpoint_remaining:
                self.checkpoint_remaining.remove(self.last_location)

            # Go to new planning
            self.plan = None

            print("Checkpoints remaining after logic: ", self.checkpoint_remaining)
            print(" ")

        instructions = Instructions()

        if self.last_lattitude is not None and self.last_longitude is not None:
            epsilon = 0.0001
            if abs(self.last_lattitude - latitude) < epsilon and self.last_longitude  - longitude < epsilon:
                # Stuck!

                self.last_lattitude = latitude
                self.last_longitude = longitude

                instructions.heading = Heading(random.uniform(0, 360))

                # go in a weird direction
                #print("Stuck! ", end="")
                return instructions

        self.last_lattitude = latitude
        self.last_longitude = longitude

        given_heading = goto(Location(latitude=latitude, longitude=longitude), location)

        instructions.heading = Heading(given_heading)

        u, v = forecast.get_uv(lat=latitude, lon=longitude, t=0)
        # u positive for west to east
        # v positive for north to south

        wind_heading = 180/np.pi * np.arctan2(v, u)
        #wind_heading = 180 / np.pi * np.arctan2(-v, u)

        if wind_heading < 0:
            wind_heading += 360

        diference = abs(given_heading - wind_heading)
        if diference > 150:

            goto_heading = Heading(given_heading + 25*self.last_wiggle_sign)

            self.last_wiggle_sign = self.last_wiggle_sign * (-1)

            print("Against the wind! goto: ", given_heading, " wind=", wind_heading, "result = ", goto_heading.angle, "u=",u, "v=", v)

            instructions.heading = goto_heading
            #return instructions
        else:
            print("Normal print out goto: ", given_heading, " wind=", wind_heading, "u=", u, "v=", v, "diference", diference)

        return instructions


        # Possible to do different course than straight to next goal
        instructions.location = location
        instructions.sail = sail

        return instructions
