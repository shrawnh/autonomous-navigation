from controller import Robot, device

class MyRobot:
    def __init__(self, robot_instance: Robot) -> None:
        self.robot_instance = robot_instance
        self.time_step = 64
        self.max_speed = 6.28
        self.turn_speed = 0.5
        self.right_motor_device: device.Device = robot_instance.getDevice("right wheel motor")
        self.left_motor_device: device.Device = robot_instance.getDevice("left wheel motor")
        self.right_speed = 0
        self.left_speed = 0
        self.distance_sensors: list[device.Device] = []
        self.dsValues = []  # distance sensors' values
        self.dsNames = ["ds0", "ds1", "ds2", "ds3", "ds12", "ds13", "ds14", "ds15"]
        self.detection_threshold = 180.0
        self._initialize_motors()
        self.enable_sensors()

    def _initialize_motors(self) -> None:
        self.left_motor_device.setPosition(float("inf"))
        self.right_motor_device.setPosition(float("inf"))
        self.left_motor_device.setVelocity(0.0)
        self.right_motor_device.setVelocity(0.0)

    def _set_speed(self, left_speed: float, right_speed: float) -> None:
        self.left_motor_device.setVelocity(left_speed)
        self.right_motor_device.setVelocity(right_speed)
        self.right_speed = right_speed
        self.left_speed = left_speed

    def enable_sensors(self) -> None:
        for name in self.dsNames:
            sensor = self.robot_instance.getDevice(name)
            sensor.enable(self.time_step)
            self.distance_sensors.append(sensor)

    def get_speed(self) -> tuple[float, float]:
        return self.left_speed, self.right_speed

    def stop(self) -> None:
        self._set_speed(0, 0)

    def move_forward(self) -> None:
        self._set_speed(
            self.max_speed * self.turn_speed, self.max_speed * self.turn_speed
        )

    def move_backward(self) -> None:
        self._set_speed(
            -self.max_speed * self.turn_speed, -self.max_speed * self.turn_speed
        )

    def turn_left(self) -> None:
        self._set_speed(
            -self.max_speed * self.turn_speed, self.max_speed * self.turn_speed
        )

    def turn_right(self) -> None:
        self._set_speed(
            self.max_speed * self.turn_speed, -self.max_speed * self.turn_speed
        )

    def obstacle_detected(self) -> bool:
        self._enable_sensors()
        self.dsValues = [
            self.distance_sensors[i].getValue() for i in range(len(self.dsNames))
        ]
        return any(dsValue > self.detection_threshold for dsValue in self.dsValues)
