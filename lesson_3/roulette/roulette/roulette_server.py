import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from roulette_interfaces.srv import Roulette
from roulette_interfaces.msg import Bet


class RouletteWheel():
    def __init__(self):
        self.win_multiplier = 36
        self.colors = ['red', 'black']
        self.numbers = np.arange(0, 37)
        self.color_win_multiplier = 2

class RouletteServer(Node):
    def __init__(self):
        super().__init__('roulette_server')
        self.declare_parameter('lucky_number', -1)
        self.srv = self.create_service(Roulette, 'roulette', self.roulette_callback)
        self.wheel = RouletteWheel()
        self.get_logger().info('Roulette server is ready.')
        self.add_on_set_parameters_callback(self.parameter_callback)


    def parameter_callback(self, params):
        for param in params:
            if param.name == 'lucky_number':
                num = param.value
                for i in range(4):
                    self.wheel.numbers = np.append(self.wheel.numbers, num)
        return SetParametersResult(successful=True)

    def roulette_callback(self, request, response):
        bet = request.bettings

        winning_number = np.random.choice(self.wheel.numbers)
        winning_color = np.random.choice(self.wheel.colors)
        response.winning_number = int(winning_number)
        response.winning_color = str(winning_color)
        if bet.number > -1:
            if bet.number == winning_number:
                winnings = bet.amount * self.wheel.win_multiplier
                response.winnings = winnings
                self.wheel.numbers = np.arange(0, 37)
            else:
                response.winnings = 0
                self.wheel.numbers = np.delete(self.wheel.numbers, np.where(self.wheel.numbers == winning_number))

        elif bet.color != '':
            if bet.color == winning_color:
                winnings = bet.amount * self.wheel.color_win_multiplier
                response.winnings = winnings
            else:
                response.winnings = 0
        rclpy.logging.get_logger('roulette_server').info(f'numbers in play: {self.wheel.numbers}')
        return response

def main(args=None):
    rclpy.init(args=args)

    roulette_server = RouletteServer()

    rclpy.spin(roulette_server)

    roulette_server.destroy_node()
    rclpy.shutdown()