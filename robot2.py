from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils
import math
import json

to_boro = False

class MyRobot2(RCJSoccerRobot):    
    def run(self):
        global to_boro

        self.team_emitter = self.robot.getDevice("team emitter")
        self.team_receiver = self.robot.getDevice("team receiver")
        self.team_receiver.enable(TIME_STEP)
        
        while self.robot.step(TIME_STEP) != -1:                    
            ballx3, bally3, robotx3, roboty3, robot_angle3, strength3, ballx2, bally2, robotx2, roboty2, robot_angle2, strength2, ballx1, bally1, robotx1, roboty1, robot_angle1, strength1, fasle_robot2_ta_robot1, fasle_robot3_ta_robot1, robot1DataValid, robot2DataValid, robot3DataValid = utils.receive(self)
            utils.sender(self)

            if self.is_new_data():
                if utils.updateSensor(self): continue
                utils.toop_be_zamin_update(self)

                if utils.globalDataValid:
                    if fasle_robot3_ta_robot1<utils.fasele_ta_robot1: to_boro = True
                    else: 
                        to_boro = False
                        if roboty1 > 0.69 and 0.23 < robotx1 < 0.35: utils.go_to(self, 0.3, 0.71)
                        elif roboty1 > 0.69 and -0.23 > robotx1 > -0.35: utils.go_to(self, -0.3, 0.71)
                        else: utils.attack(self)
                else: utils.go_to(self, -0.4, 0.3)
                self.send_data_to_team(self.player_id)
