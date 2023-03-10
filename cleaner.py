import os

path_to_install_dir = os.getcwd() + "/install"

source_files = []

for root, dirs, files in os.walk(path_to_install_dir):
    for file in files:
        if file.endswith(".py"):
            path_to_file = os.path.join(root, file)
            if ('/lib/python3.10/site-packages/' in path_to_file and
                'dobot_driver' not in path_to_file and 
                'dobot_motion' not in path_to_file and 
                'dobot_msgs' not in path_to_file):

                os.remove(os.path.join(root, file))

print('Source files in .py format removed.')
