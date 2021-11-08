#configuration file

class Interp_config:
    def __init__(self):
        self.board_shape = [8,5] # height , width
        self.board_size = self.board_shape[0] * self.board_shape[1]
        return