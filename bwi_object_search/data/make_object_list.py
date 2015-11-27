

def main():
    num_tag = 40
    with open("temp_object_list.txt", 'w') as f:
        f.write("# The number of patterns to be recognized\n")
        f.write(str(num_tag))
        f.write("\n\n")

        for i in range(1, num_tag + 1):
            f.write("# Object number {}\n".format(i))
            f.write("YAY{}\n".format(i))
            f.write("data/4x4/4x4_{}.patt\n".format(i))
            f.write("80.0\n")
            f.write("0.0 0.0\n\n")


if __name__ == "__main__":
    main()
