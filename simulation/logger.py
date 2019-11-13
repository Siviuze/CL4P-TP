# A basic logger class compatible with the miam_plot tool.

import csv

class Logger():
    def __init__(self, field_names):
        self.data = {}
        for f in field_names:
            self.data[f] = [0.0]
        self.data["time"] = [0.0]

    def set(self, field_name, value):
        self.data[field_name][-1] = value

    def set_vector(self, field_name, value):
        self.data[field_name + "X"][-1] = value[0, 0]
        self.data[field_name + "Y"][-1] = value[1, 0]
        self.data[field_name + "Z"][-1] = value[2, 0]

    def new_line(self):
        for f in self.data.keys():
            self.data[f].append(0.0)

    def save(self, filename, description="Log from BB8 simulation"):
        header_list = ["time"]
        for f in self.data.keys():
            if f is not "time":
                header_list.append(f)
        with open(filename, 'w') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(['Robot log: BB8', description])
            writer.writerow(header_list)
            for i in range(len(self.data["time"]) - 1):
                data_line = [self.data[h][i] for h in header_list]
                writer.writerow(data_line)


# Testing

if __name__ == "__main__":
    data = ["header", "secondHeader"]
    logger = Logger(data)
    for i in range(5):
        logger.set("time", i)
        logger.set("header", 2 * i)
        logger.set("secondHeader", - i)
        logger.new_line()
    logger.save("/tmp/test.csv")
