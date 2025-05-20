To create a virtual environment, open a terminal window and navigate to the directory that you cant to place it. Then type:
`python3 -m venv gymnasiumvenv`

This will set up a virtual environment, and create a folder named "gymnasiumvenv" with subfolders and files.

To activate the virtual environment, you have to type:
`source gymnasiumvenv/bin/activate`

After that, the terminal will change to indicate that the virtual environment is active:
`(gymnasiumvenv) ... $`

Now that your virtual environment is active, you can install packages using pip3. To install, for example, the Crazyflie client simply type:
`(gymnasiumvenv) ... $ pip3 install cfclient`