import numpy as np
import rclpy
from rclpy.node import Node
from roulette_interfaces.srv import Roulette
from roulette_interfaces.msg import Bet
import time


class RouletteClient(Node):
    def __init__(self):
        super().__init__('roulette_client')
        self.cli = self.create_client(Roulette, 'roulette')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Roulette.Request()
        self.bank = 1000  # Initial bank amount
        self.player_funds = 100  # Initial player funds

    def place_bet(self):
        print(f'You have {self.player_funds} units available to bet.')
        bet_type = input('Do you want to bet on a number or a color? (n/c): ').strip().lower()
        if bet_type == 'n':
            number = int(input('Enter the number you want to bet on (0-36): '))
            amount = int(input('Enter the amount you want to bet: '))
            if amount > self.player_funds:
                print('Insufficient funds for this bet.')
                return None
            self.player_funds -= amount
            return Bet(number=number, color='', amount=amount)
        elif bet_type == 'c':
            color = input('Enter the color you want to bet on (red/black): ').strip().lower()
            amount = int(input('Enter the amount you want to bet: '))
            if amount > self.player_funds:
                print('Insufficient funds for this bet.')
                return None
            self.player_funds -= amount
            return Bet(number=-1, color=color, amount=amount)
        else:
            print('Invalid bet type.')
            return None

    def send_request(self, bet):
        self.req.bettings = bet
        self.future = self.cli.call_async(self.req)

    def game_loop(self):
        while True:
            bet = self.place_bet()
            if bet is None:
                continue
            self.send_request(bet)
            rclpy.spin_until_future_complete(self, self.future)
            if self.future.result() is not None:
                winnings = self.future.result().winnings
                winning_number = self.future.result().winning_number
                winning_color = self.future.result().winning_color
                print(f'The winning number is {winning_number} ({winning_color}).')
                self.bank += bet.amount
                if winnings > 0:
                    print(f'You won {winnings} units!')
                    self.player_funds += winnings
                    self.bank -= winnings
                else:
                    print('You lost your bet.')
                print(f'The bank now has {self.bank} units.')
                print(f'Your current funds: {self.player_funds} units.')
            else:
                self.get_logger().error('Service call failed %r' % (self.future.exception(),))
            if self.player_funds <= 0:
                print('You have run out of funds. You lost! Game over.')
                time.sleep(10)
                break
            elif self.bank <= 0:
                print('The bank has run out of funds. You won! Game over.')
                time.sleep(10)
                break

def main(args=None):
    rclpy.init(args=args)
    roulette_client = RouletteClient()
    try:
        roulette_client.game_loop()
    except KeyboardInterrupt:
        print('Game interrupted. Exiting...')
    finally:
        roulette_client.destroy_node()
        rclpy.shutdown()