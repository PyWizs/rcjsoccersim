from rcj_soccer_robot import TIME_STEP
import math
import json

ASHAR, state, fasele_ta_robot1, robotx, roboty, heading, robot_angle, direction, strength, xb, yb, ball_x, ball_y, ball_dist, zavie_maghsad, error_zavie, error_fasele, toop_be_zamin_x, toop_be_zamin_y, dist_ta_darvaze = 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
robot_pos, data, team_data, ball_data = "", "", "", ""
ball_is_available, isTurning, robot1DataValid, robot2DataValid, robot3DataValid, globalDataValid = False, False, False, False, False, False
ballx3, bally3, robotx3, roboty3, robot_angle3, strength3, ballx2, bally2, robotx2, roboty2, robot_angle2, strength2, ballx1, bally1, robotx1, roboty1, robot_angle1, strength1, fasle_robot2_ta_robot1, fasle_robot3_ta_robot1 = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0


def updateSensor(self):
    global robotx, roboty, robot_pos, data, team_data, ball_data, heading, sonar_values, robot_angle, ball_x, ball_y, ball_dist, ball_angle, direction, strength, ball_is_available, zavie_toop_be_robot, dist_ta_darvaze, fasele_ta_robot1, globalDataValid

    data = self.get_new_data()  

    while self.is_new_team_data(): team_data = self.get_new_team_data()  
    
    heading = self.get_compass_heading() 
    robot_angle=math.degrees(heading)

    robot_pos = self.get_gps_coordinates()  

    if self.name[0] == "B":
        robotx = robot_pos[0]
        roboty = robot_pos[1]
    else:
        robotx = -robot_pos[0]
        roboty = -robot_pos[1]
    
    sonar_values = self.get_sonar_values()  

    if self.is_new_ball_data():
        ball_data = self.get_new_ball_data()
        ball_x = ball_data["direction"][0]
        ball_y = ball_data["direction"][1]
        direction = get_direction(ball_data["direction"])
        strength = ball_data["strength"]
        ball_is_available = True
    else:
        ball_data = None
        direction = None
        ball_is_available = False

    ball_angle = math.degrees(math.atan2(ball_y, ball_x))
    
    zavie_toop_be_robot = (ball_angle+robot_angle) % 360

    dist_ta_darvaze = math.sqrt((robotx-0)**2+(roboty-0.7)**2)

    if self.player_id != 1: fasele_ta_robot1 = math.sqrt((robotx1-robotx)**2 + (roboty1-roboty)**2)

    globalDataValid = robot1DataValid or robot2DataValid or robot3DataValid or ball_is_available


def setMotorSpeed(self, left_speed, right_speed):
    left_speed,right_speed = right_speed,left_speed
    left_speed = min(max(left_speed, -10) , 10)
    right_speed = min(max(right_speed, -10) , 10)
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)


def setAngleBot(self, darage: int):
    while self.robot.step(TIME_STEP) != -1:
        heading = self.get_compass_heading()
        darageBot = math.degrees(heading)

        if(darage - 4 < darageBot < darage + 4): setMotorSpeed(self, 0, 0); break

        # miangin = darage - darageBot
        miangin = darage

        if(miangin < 0): setMotorSpeed(self, -4, 4)
        else: setMotorSpeed(self, 4, -4)


def defeandFast(self, pos, angle):
    if (pos[0] + 0.05 > robot_pos[0] > pos[0] - 0.05) and (pos[1] + 0.05 > robot_pos[1] > pos[1] - 0.05): setAngleBot(self, angle)
    else: go_to(self, pos[0], pos[1])


def get_direction(ball_vector):
    if -0.13 <= ball_vector[1] <= 0.13: return 0
    return -1 if ball_vector[1] < 0 else 1


def go_to(self, x_maghsad, y_maghsad):
    global zavie_maghsad, error_zavie, error_fasele, delta_x, delta_y

    delta_x = x_maghsad - robot_pos[0]
    delta_y = y_maghsad - robot_pos[1]

    zavie_maghsad = math.atan2(delta_y, delta_x) * 57.29577951308232

    zavie_maghsad = (zavie_maghsad - 90)
    if zavie_maghsad < 0: zavie_maghsad += 360

    error_zavie = (zavie_maghsad - robot_angle) - 180
    
    if error_zavie > 180: error_zavie -= 360
    elif error_zavie < -180: error_zavie += 360
    error_zavie = error_zavie * 3

    error_fasele = math.sqrt((delta_x)**2 + (delta_y)**2)
    
    error_fasele = round(error_fasele * 180,3) if error_fasele < 0.5 else round(error_fasele * 50,3)
    if error_fasele>1:setMotorSpeed(self,error_fasele - error_zavie+10, error_fasele + error_zavie+10)
    else: setMotorSpeed(self,0,0)


def toop_be_zamin_update(self):
    global toop_be_zamin_x, toop_be_zamin_y, ball_dist, ball_angle

    if ball_is_available:
        r = 0.11886 * ball_dist - 0.02715

        ball_angle = math.degrees(math.atan2(ball_y, ball_x))

        if ball_angle < 0: ball_angle += 360
        theta = (ball_angle+robot_angle+90) % 360
        if theta < 0: theta += 360

        theta = math.radians(theta)
        pos_x = round(r * math.cos(theta), ASHAR)
        pos_y = round(r * math.sin(theta), ASHAR)        
        
        toop_be_zamin_x = round(robotx + 2.5 * pos_x, ASHAR)
        toop_be_zamin_y = round(roboty + 2.5 * pos_y, ASHAR)
    else:
        otherBallX, numberOfValdDataX, otherBallY, numberOfValdDataY = 0, 0, 0, 0

        for (ballx, bally), valid in zip([(ballx1, bally1), (ballx2, bally2), (ballx3, bally3)], [robot1DataValid, robot2DataValid, robot3DataValid]):
            if valid: 
                otherBallX += ballx; numberOfValdDataX += 1
                otherBallY += bally; numberOfValdDataY += 1
        
        toop_be_zamin_x = otherBallX / numberOfValdDataX if numberOfValdDataX > 0 else None
        toop_be_zamin_y = otherBallY / numberOfValdDataY if numberOfValdDataY > 0 else None


def goDefend(self):
    if globalDataValid:
        if toop_be_zamin_y < roboty:
            if 0 < robotx < 0.33 or -0.33 < robotx <0: go_to(self, toop_be_zamin_x, 0.6)
            elif robotx >= 0.33: go_to(self, 0.33, 0.6)
            elif robotx <= -0.33: go_to(self, -0.33, 0.6)
        elif toop_be_zamin_y > roboty and robotx > 0: go_to(self, 0.33, toop_be_zamin_y)
        elif toop_be_zamin_y > roboty and robotx < 0: go_to(self, -0.33, toop_be_zamin_y)
    else: go_to(self, 0, 0.6)


def allowToGo(self, robotx, roboty):
    if roboty <= 0.6: return True
    else: go_to(self, robotx, 0.55); return False


def attack(self):
    global state, isTurning

    if not allowToGo(self, robotx, roboty): return

    if ball_is_available:
        if toop_be_zamin_y > roboty: isTurning = True
        elif zavie_toop_be_robot > 350 or zavie_toop_be_robot < 10: isTurning = False

        if not isTurning: goToTheta(self, zavie_toop_be_robot-90)
        else:
            if toop_be_zamin_x < robotx: goToTheta(self, zavie_toop_be_robot-90 - strength / 1.5)
            else: goToTheta(self, zavie_toop_be_robot-90 + strength / 1.5)
    else: go_to(self, toop_be_zamin_x, toop_be_zamin_y)


def attack2(self):
    global state, isTurning

    if toop_be_zamin_y > roboty: isTurning = True
    elif zavie_toop_be_robot > 350 or zavie_toop_be_robot < 10: isTurning = False

    if toop_be_zamin_y < roboty:
        if state == 1:
            go_to(self, toop_be_zamin_x, toop_be_zamin_y+0.045)
            if abs(toop_be_zamin_x-robotx) < 0.01 and abs((toop_be_zamin_y+0.045)-roboty) < 0.01: state=2
        elif state == 2:
            go_to(self, toop_be_zamin_x, toop_be_zamin_y)
            if abs(toop_be_zamin_x-robotx) > 0.2 and abs(toop_be_zamin_y-roboty) > 0.2: state=1
    else:
        jahat = toop_be_zamin_x - 0.05 if toop_be_zamin_x - robotx > 0 else toop_be_zamin_x + 0.05
        go_to(self, jahat, toop_be_zamin_y + 0.045)


def goToTheta(self,moveTheta):
    go_to(self,robotx+math.cos(moveTheta*0.0174532925199433)*0.05,roboty+math.sin(moveTheta*0.0174532925199433)*0.05)


def receive(self):
    global ballx3, bally3, robotx3, roboty3, robot_angle3, strength3, ballx2, bally2, robotx2, roboty2, robot_angle2, strength2, ballx1, bally1, robotx1, roboty1, robot_angle1, strength1, fasle_robot2_ta_robot1, fasle_robot3_ta_robot1, robot1DataValid, robot2DataValid, robot3DataValid
    while self.team_receiver.getQueueLength() > 0:
        packet = self.team_receiver.getString()
        self.team_receiver.nextPacket()

        data = json.loads(packet)
        
        for key, value in data.items():
            if key == "robot_num": robot_num = value

            elif robot_num == 1:
                if key == "toop_be_zamin_x": ballx1 = value
                elif key == "toop_be_zamin_y": bally1 = value
                elif key == "robotx": robotx1 = value
                elif key == "roboty": roboty1 = value
                elif key == "robot_angle": robot_angle1 = value
                elif key == "strength": strength1 = value
                elif key == "ball_is_available": robot1DataValid = value  
            
            elif robot_num==2:
                if key == "toop_be_zamin_x": ballx2 = value
                elif key == "toop_be_zamin_y": bally2 = value
                elif key == "robotx": robotx2 = value
                elif key == "roboty": roboty2 = value
                elif key == "robot_angle": robot_angle2 = value
                elif key == "strength": strength2 = value
                elif key == "fasle_robot2_ta_robot1": fasle_robot2_ta_robot1 = value
                elif key == "ball_is_available": robot2DataValid = value  
            
            elif robot_num == 3:
                if key == "toop_be_zamin_x": ballx3 = value
                elif key == "toop_be_zamin_y": bally3 = value
                elif key == "robotx": robotx3 = value
                elif key == "roboty": roboty3 = value
                elif key == "robot_angle": robot_angle3 = value
                elif key == "strength": strength3 = value
                elif key == "fasle_robot3_ta_robot1": fasle_robot3_ta_robot1 = value
                elif key == "ball_is_available": robot3DataValid = value

    return ballx3, bally3, robotx3, roboty3, robot_angle3, strength3, ballx2, bally2, robotx2, roboty2, robot_angle2, strength2, ballx1, bally1, robotx1, roboty1, robot_angle1, strength1, fasle_robot2_ta_robot1, fasle_robot3_ta_robot1, robot1DataValid, robot2DataValid, robot3DataValid


def sender(self):
    if ball_is_available: data = {"robot_num": self.player_id, "robotx": robotx, "roboty": roboty, "robot_angle": robot_angle, "strength": strength, "toop_be_zamin_x": toop_be_zamin_x, "toop_be_zamin_y": toop_be_zamin_y, "ball_is_available": ball_is_available, f"fasle_robot{self.player_id}_ta_robot1": fasele_ta_robot1} 
    else: data = {"robot_num": self.player_id, "robotx": robotx, "roboty": roboty, "robot_angle": robot_angle, "strength": strength, "ball_is_available": ball_is_available}  
    packet = json.dumps(data)
    self.team_emitter.send(packet)