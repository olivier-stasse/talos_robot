import os


def add_to_files(line_to_add, file_name, folder):
    f = open(f"{folder}/{file_name}", "r")
    content = ""
    for line in f:
        content += line

    f.close()

    f = open(f"{folder}/{file_name}", "w")

    content_lines = content.split("\n")

    f.write(line_to_add + '\n')
    for l in content_lines:
        msg = "\t" + l + "\n"
        f.write(msg)

    f.close()


line_to_add = "ros__parameters:"
folder = "local_joint_control/inertia_shaping_controller"
for filename in os.listdir(folder):
    add_to_files(line_to_add, filename, folder)
