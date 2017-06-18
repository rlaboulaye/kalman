import asyncio
import json
from time import time
from time import sleep


def average(values):
    total = 0
    for n in values:
        total += n
    avg = total / len(values)
    return avg

def main(host, port):

    # Setup asyncio and open the connection
    loop = asyncio.get_event_loop()
    reader, writer = loop.run_until_complete(
        asyncio.open_connection(host, port))

    # Simple method to process a command. It's defined inside the main
    # method so reader and writer are in its namespace
    def do(command):
        print('>>>', command)

        # Send the command -- write() expect bytes, so we call encode()
        writer.write(command.strip().encode())

        # This is a lot in one line, but here's what's happening:
        #   reader.readline() starts an asyncio coroutine that reads
        #     until it gets a complete line (ending with \n) and then
        #     returns that coroutine.
        #   run_until_complete() runs the coroutine until it terminates
        #   decode() turns the bytes object into a string
        #   strip() removes whitespace at the beginning or end, like \n
        res = loop.run_until_complete(reader.readline()).decode().strip()
        print('<<<', res)
        try:
            # The response is a json encoded string, so we decode it
            res = json.loads(res)
        except json.decoder.JSONDecodeError:
            # If an error occurred, handle it gracefully
            print('Error decoding response')
            res = {}
        print()
        # Return the resulting dict
        return res


    x_velocities = []
    y_velocities = []

    do('speed 10 10')
    # loop.run_until_complete(asyncio.sleep(1.0))

    for i in range(0, 5):

        sleep(0.5)

        robot_dict = do('where robot')
        start_position = robot_dict['center']

        start_time = time()

        sleep(1.0)

        robot_dict = do('where robot')
        end_position = robot_dict['center']

        end_time = time()

        delta_x = end_position[0] - start_position[0]
        delta_y = end_position[1] - start_position[1]
        delta_t = end_time - start_time

        x_velocities.append(delta_x / delta_t)
        y_velocities.append(delta_y / delta_t)

    do('speed 0 0')

    avg_delta_x = average(x_velocities)
    avg_delta_y = average(y_velocities)

    print()
    print('average x velocity: {} pixels/sec'.format(avg_delta_x))
    print('average y velocity: {} pixels/sec'.format(avg_delta_y))
    print()

    # Close the connection
    writer.close()


if __name__ == '__main__':
    from sys import argv
    main(*argv[1:])
