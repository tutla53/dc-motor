# Motor Control Plotter Config

group_name = {
                "open"  : "Open_Loop_",
                "speed" : "Speed_Step_",
                "pos"   : "Position_Step_",
            }

command_header = {
                "open"  : "Commanded_PWM",
                "speed" : "Commanded_Speed(RPM)",
                "pos"   : "Commanded_Position(rotation)",
                    }

data_header = {
                "open"  : "Motor_Speed(RPM)",
                "speed" : "Motor_Speed(RPM)",
                "pos"   : "Motor_Position(rotation)",
            }

plot_title = {
                "open"  : "Open Loop Response",
                "speed" : "Speed Control Response",
                "pos"   : "Position Control Response",
            }

y_label = {
                "open"  : "Motor Speed(RPM)",
                "speed" : "Motor Speed(RPM)",
                "pos"   : "Motor Position (Rotation)",
            }

data_legend = {
                "open"  : "Actual Open Loop Response",
                "speed" : "Actual Motor Response",
                "pos"   : "Actual Motor Response",
            }

simulation_legend = {
                "open"  : "Open Loop Simulation",
                "speed" : "Speed Simulation",
                "pos"   : "Position Simulation",
            }

commanded_legend = {
                "open"  : "PWM Input",
                "speed" : "Commanded Speed",
                "pos"   : "Commanded Position",
            }

same_unit = {
                "open"  : False,
                "speed" : True,
                "pos"   : True,
            }
