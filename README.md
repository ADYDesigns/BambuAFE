# Bambu Automatic Fume Extractor (BambuAFE)
This project intends to reduce the amount of harmful gases produced during the 3D printing process by creating a slight negative pressure in the chamber to reduce or eliminate them leaking out.  It does this by automatically controlling a fume extractor fan based on what your Bambu printers are doing.  It uses a 3D printed filter box with active charcoal and an ESP32 to poll one or two Bambu printers for status information and ramp a set of fans up and down based on the printer status.  It also gives basic printer status information through a dashboard.

Although this project was designed around two printers it can easily be used for one printer with no modifications other than printing a cap for the non-used intake.  Using more than two printers is also possible but is an advanced use case requiring some modifications to the code or possibly the 3D printed model and is not required for a standard one or two printer setup.

# Bill of Materials
This is a list of materials that will be required.  These are my suggestions to have a successful project but if you are already experienced with electronics and microcontrollers you can of course change the items as you see fit.

- **Filter Box** - The filter box itself that will house all the other parts.  Download and print from MakerWorld: https://makerworld.com/en/@ADYDesigns/upload
- **Exhaust System** - You will need a way to get the exhaust from your printer(s) over to this system.  I highly recommend https://makerworld.com/en/models/1880330-slim-h2-quicklock-exhaust-system-updated#profileId-2013378 as my system is designed to directly use that system's hose adapters.  In fact the threads on the front and back of my model are being used with permission from that project.
- **ESP32-WROOM controller** - Any ESP32 board should work but the board I'm using is the HiLetgo ESP-WROOM-32 development board with the pins on the bottom: https://www.amazon.com/dp/B0718T232Z
- **Breakout board** - If you are comfortable soldering you don't need this but it makes wiring and mounting much easier: https://www.amazon.com/dp/B0BYS6THLF
- **12 volt power supply** - Anything to cover the power usage of the ESP32 and the fans which combined at max power shouldn't be more than about 1.5A: https://www.amazon.com/dp/B07VQHCK6P
- **Panel mount barrel connectors** - Not required but makes the connection through the box a lot cleaner: https://www.amazon.com/dp/B0DLKN8J7M
- **12v to 5v converter** - To power the ESP32 off of the 12v power adapter so you don't have to have two power sources: https://www.amazon.com/dp/B0B6NZBWV4
- **Wire connectors** - You can also solder or wire nut the wires but this simplifies the wiring: https://www.amazon.com/dp/B0DGPXWJ28
- **Fans** - Any high static pressure 92mm fan will work such as the Arctic P9 Max or Noctua NF-A9 PWM.  I went with the Arctic P9's: https://www.amazon.com/dp/B0D4YZFKP5
- **Pre-Filter** - This is just to keep dust out of the active carbon.  Almost any thin filter material will do: https://www.lowes.com/pd/Frost-King-Common-15-in-x-24-in-x-0-1875-in-Actual-15-in-x-24-in-Washable-Cut-To-Fit-Air-Filter/1196229
- **Active Carbon** - This is doing the majority of the work here and is the most important part.  You can buy loose active carbon from multiple sources such as a pet store but I went with one made for air filtration: https://www.amazon.com/dp/B0CFCKFPHR
  * Note: Most active carbon should be washed and fully dried before being used.  Check the instructions on the packaging.
- **Filter Bag** - A bag to hold the active carbon: https://www.amazon.com/dp/B09HQFZXSY
- **Misc Hardware** - 10x M3x6 self tapping screws, 4x M4x30 screws & washers per fan, and a USB to Micro USB cable to connect the ESP32 directly to your computer.

# Assembly Guide
This assembly guide is assuming you have everything from the bill of materials above.  If you made any changes please follow the instructions for your equipment.

1. Take the 3D printed filter box and start attaching your components:
   1. Mount the breakout board to the box using 4x M3x6 self tapping screws.  Make sure GND is at the top right and 5v is at the bottom left.
   2. Mount the panel mounted power connector through the hole under the breakout board using its supplied washer and nut.
   3. Mount the two wire connectors above the power connector using 4x M3x6 self tapping screws.
      * Note: You may want to skip these and attach them after you run the wires.
   4. Mount the 12v to 5v converter under the breakout board using 2x M3x6 self tapping screws.
   5. Mount the fan(s) to the front using 4x M4x30 screws with washers.  Make sure they are exhausting out the front by looking at the blades.  They "scoop" the air when rotating.  For the Arctic P9 the air flow should be from the logo side to the non-logo side (logo will face inside the box).  This can always be flipped later.
2. Wire each of the components.
   1. Wire the panel mounted power connector to the wire connectors.  Shorten the wires as needed or coil them up.  At this point you may want to label the power connectors P and N or + and -.
   2. Wire the 12v to 5v converter to the power connectors.  In general Red is Positive (+) and Black is Negative (-) — check the labeling on your converter to confirm.
   3. Wire the fans' power wires to the power connectors. Depending on the fans you bought the connector should be labeled. Pin 1 should be ground (to the N or - power connector), Pin 2 should be +12v (to the P or + power connector), Pin 3 is not used, and Pin 4 is our PWM. Verify this information on the packaging or the manufacturer's website. These wires are small but the power connectors should hold them. If you have trouble carefully strip 1/2 an inch of insulation off the end then double the bare wire back onto itself and wrap it around to make it thicker.
      * Note: If you want you can use Pin 3 to pull fan speed back in and display it but currently that hasn't been implemented.
   5. Wire the fans' PWM wire to the breakout board.  Each fan gets its own wire — by default fan 1 uses GPIO16 and fan 2 uses GPIO17, which should be labeled as 16 and 17 on the breakout board.  The order doesn't matter as they both come on at the same time.
   6. Mount the ESP32 to the breakout board.  Verify it matches the breakout board and in general the USB port should be facing down.
3. Cut the pre-filter to size and slide it into the small channel nearest the back of the box.
4. Fill the media bag with your active carbon, checking fullness against the channel.  You want it full but not tightly packed.
5. ⚠ Safety Note: Make sure to print out a mesh cap for each exhaust so you don't accidentally stick your fingers in the fan blades — End Cap with Gyroid Mesh (https://makerworld.com/en/models/1880330-slim-h2-quicklock-exhaust-system-updated#profileId-2606325).
6. If you are hooking up to only one printer also print out a cap for the second intake — Solid End Cap (No Airflow) (https://makerworld.com/en/models/1880330-slim-h2-quicklock-exhaust-system-updated?profiled=2938064#profileId-2938064).

# Installing the Software and Flashing the Controller
For the programming side I will be using the free Microsoft Visual Studio Code program available at https://code.visualstudio.com/ .  You can also use the Arduino IDE if you are familiar with that but these instructions will use VS Code.

These instructions assume a standard Windows 10+ machine.  Please adjust for the system you are using.

Before plugging in the ESP32 you may need to install a USB driver so your computer can communicate with it.  The HiLetgo board uses a CP2102 USB chip.  Download and install the driver from https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads before continuing.  Without this driver Windows may not recognise the board when you plug it in.

1. Open VS Code and then press CTRL+SHIFT+X or click the blocks/squares icon on the left sidebar.  In the Extensions search box enter PlatformIO and select 'PlatformIO IDE' and install it.  Once installed you may need to restart VS Code before the PlatformIO icon appears in the left sidebar.
2. Download the code for this project by clicking the green 'Code' button on the GitHub project page and selecting 'Download ZIP'.
3. Unzip the zip file by right clicking it and selecting 'Extract all'.
4. In VS Code select File -> Open Folder and select the folder you extracted the files to.
   * You will be prompted if you want to trust the author of the files.  Click 'Yes, I trust the authors'.
5. PlatformIO will configure the project and download any supporting files it needs.  The first time you do this may take a few minutes.
6. Plug your ESP32 into your computer using a USB to Micro USB cable.
7. Upload the web interface files by clicking on the PlatformIO icon indicated by an alien head on the left sidebar then General -> Upload Filesystem Image.  This uploads the web pages (dashboard, setup form, etc.) that run on the controller and must be done before flashing the firmware.  Make sure the ESP32 is connected and recognised by your computer before doing this — check Device Manager if you are unsure.
   * By default VS Code will try to auto detect the ESP32.  If you have other serial devices plugged in this may fail.  At the bottom of the VS Code screen you can click the power plug icon that says "Auto" next to it and select the ESP32 directly.
   * If the upload fails you may have to put the ESP32 into boot mode.  Unplug the power, wait a few seconds, then while holding the boot button down plug the power back in.  Count to at least 3 then let go of the boot button and retry the upload.
8. Flash the firmware by clicking on the PlatformIO icon indicated by an alien head on the left sidebar then General -> Upload.  You can also press CTRL+ALT+U as a shortcut though this may not work on all keyboards or systems.
   * After flashing, open the Serial Monitor (PlatformIO -> Monitor) to watch the status of your controller.

# Initial Setup of the Controller
1. By default the ESP32 will broadcast a wireless network called 'BambuAFE-Setup'.  Look for and connect to this WiFi network.
2. Once connected open up a web browser and navigate to the default webpage at 'http://192.168.4.1/'.
3. Enter your WiFi SSID (your WiFi network name), WiFi password, a device name, and a password that will be used to access the device in the future, then click 'Save'.
   * Leave the device name as BambuAFE-ESP32 unless you have a good reason to change it such as you have multiple controllers and want to make them BambuAFE-1 and BambuAFE-2.  Keep the name to 15 characters or less as this is a technical limit to the hostname.
4. The ESP32 will reboot and connect to your WiFi.  You will see this in the monitor within VS Code.  If it does not connect see the section 'Resetting the Config'.

# Configuring the AFE
1. The controller should now be on your network.  Use either the controller's IP address such as 'http://192.168.1.162/' or the hostname such as 'http://BambuAFE-ESP32.local/'.  If the `.local` address does not work in your browser use the IP address instead — the IP address is always printed to the Serial Monitor when the controller boots.
2. Enter `admin` as the username — this is the default hardcoded username for the controller and cannot be changed — and the password you configured during setup.
3. By default you will get a status page.  Click the '⚙ Configuration' link at the top to change settings.
4. Under the **Bambu Cloud Account** section you will need to sign in to link the controller to your Bambu account.  There are three ways to do this:
   1. **Option A — PowerShell script:** This is the recommended way for Windows users.
      1. Download `get_bambu_token.ps1` from the Configuration page on the controller — there is a download link in the Bambu Cloud Account section.
      2. Open a [PowerShell 7](https://github.com/powershell/powershell/releases) window and navigate to the directory you saved the script to and execute it by typing `get_bambu_token.ps1` and pressing enter.
      3. Edge will open automatically — log in to your Bambu account and complete any Cloudflare check or 2FA if prompted.
      4. While leaving Edge open return to the PowerShell window and press ENTER once you are fully logged in.
      5. The script will extract your token and offer to send it directly to the controller — enter your dashboard password when prompted.
      6. Return to the configuration page and press F5 to refresh.  It should indicate that the token is saved with a green checkmark.
   2. **Option B — Manual token entry:** This is a manual way to find your information within the browser cookies on your machine.
      1. Log into https://bambulab.com in your browser.
      2. Press F12 to open Developer Tools.
      3. Click the **Storage** tab (Firefox) or **Application** tab (Chrome/Edge).
      4. Expand **Cookies** and click on **bambulab.com**.
      5. Find the cookie named **token** — copy its full value (a very long string).
      6. Find the cookie named **user_id** — copy its value.
      7. Paste both into the manual entry fields on the configuration page.
   3. **Option C — Python Script:**
      There are multiple Python scripts available that can retrieve your User ID and token automatically.  These scripts impersonate a web browser to bypass Cloudflare's bot detection.  Here are a couple of options:
      - https://github.com/coelacant1/Bambu-Lab-Cloud-API#quick-start-authentication
      - https://github.com/Keralots/BambuHelper/blob/main/tools/get_token.py

* Note: The access token expires approximately every 3 months.  When it expires repeat the same method to get a new one.

5. For each printer:
   1. Enter the printer's name.  This is for your own information and to see status later.
   2. Enter the printer's serial number.  This can be found on a sticker on the back or bottom of the printer, or in Bambu Studio under the Device tab.
   3. Select the printer generation.  Gen 1 covers the X1C, P1S, P1P, A1, and A1 Mini.  Gen 2 covers the H2C, H2S, H2D, and P2S.
6. On the same Configuration page scroll down to the **Fans** section:
   1. Enter the minimum speed when either printer is printing.  The fans have a minimum so 8% is usually the lowest this should be set.
   2. Enter the maximum speed when one printer is actively exhausting (when the printer's exhaust fan is actively running).  The actual fan speed will scale between the minimum and this maximum based on the printer's exhaust speed.  This should be around 50%.
   3. Enter the maximum speed when two printers are actively exhausting.  The actual fan speed will scale between the minimum and this maximum based on the average exhaust level of both printers.  This should be around 100%.
7. Click Save and the controller will apply your changes.
8. Once everything is working, close up the filter box.
   
# Resetting the Config
To reset and clear the controller, you can:
1. Unplug the controller then hold the boot button — labeled BOOT on the board — for 3 seconds during power-on.
2. Go into the Configuration screen, scroll to the Advanced section at the bottom, and click the 'Reset configuration…' button.

Once that is done you will have to go back through the **Initial Setup of the Controller** and **Configuring the AFE** sections above.
