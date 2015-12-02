def main():
    num_tag = 40
    with open("object_list.txt", 'w') as f:
        f.write("# The number of patterns to be recognized\n")
        f.write(str(num_tag))
        f.write("\n\n")

        for i in range(1, num_tag + 1):
            f.write("# Object number {}\n".format(i))
            f.write("AR_OBJECT_{}\n".format(i))          # Name of object
            f.write("data/4x4/4x4_{}.patt\n".format(i))  # File
            f.write("105.0\n")                           # Marker width, used to calculate distance
            f.write("0.0 0.0\n\n")                       # Coordinate of the center


if __name__ == "__main__":
    main()
