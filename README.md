# ardupilot-webots
ardupilot sitl for webots

#para iniciar o sitl
$HOME/eletroquad/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=$HOME/eletroquad/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm

python
    def send_rc_command(channel, pwm):
        """Send an RC override command to a specific channel."""
        rc_channels = [1500] * 8  # Default neutral values for all 8 channels
        rc_channels[channel - 1] = pwm  # Set desired PWM for the selected channel
        
        master.mav.rc_channels_override_send(
            master.target_system,  # Target system
            master.target_component,  # Target component
            *rc_channels  # RC channels list
        )
