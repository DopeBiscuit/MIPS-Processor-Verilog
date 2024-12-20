# Sieve code that calculates the first 1000 prime numbers and returns an array of all of them
def sieve():
    primes = []
    i = 2
    while len(primes) < 1000:
        is_prime = True
        for prime in primes:
            if i % prime == 0:
                is_prime = False
                break
        if is_prime:
            primes.append(i)
        i += 1
    return primes


# function implements go-back-n algorithm in the server
def go_back_n():
    # create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # bind the socket to the address
    server_socket.bind(('random address', 12345))
    # set the timeout for the server
    server_socket.settimeout(5)
    # create a buffer for the server
    buffer = []
    # create a variable to keep track of the sequence number
    sequence_number = 0
    # create a variable to keep track of the expected sequence number
    expected_sequence_number = 0
    # create a variable to keep track of the window size
    window_size = 4
    # create a variable to keep track of the base
    base = 0
    # create a variable to keep track of the number of packets received
    packets_received = 0
    # create a variable to keep track of the number of packets sent
    packets_sent = 0
    # create a variable to keep track of the number of packets dropped
    packets_dropped = 0
    # create a variable to keep track of the number of packets acknowledged
    