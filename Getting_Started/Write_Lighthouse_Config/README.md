# Write Lighthouse Configuration

The purpose of `write-config-all.py` is to write and save the Lighthouse positioning system's configuration on every Crazyflie available.
This saves time compared to uploading it to each Crazyflie manually.


- Step1: Perform the geometry estimation of your system through the cfclient and save the `.yaml` file.
- Step2: Add all the Crazyflie addresses to the `uris` list in the [write-config-all.py](Getting_Started/Write_Lighthouse_Config/write-config-all.py) script.
- Step3: Turn on all the Crazyflies and make sure they have their Lighthouse decks attached.
- Step4: Run the `write-config-all.py` script using the correct `.yaml` file: 
```
python3 write-config-all.py Lighthouse_x4.yaml
```

The script will connect to each available Crazyflie in sequence and turn it off if the upload was successful.
You can also check which uploads were successful in the terminal, as they are marked in green.
Messages like these should be ignored: 
```
Got link error callback [Too many packets lost] in state [1]
```
They appear when the computer fails to connect to a Crazyflie or when a connected Crazyflie is turned off.
