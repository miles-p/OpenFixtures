# Import library for fixture control
library = "#import <OpenFixtures.h>"

# Descriptions of different types of fixtures
fixturesDesc = [
    "Dimmer (Analog 1ch Control)",                   # Dimmer with 1 channel analog control
    "RGB (Analog 3ch Control)",                      # RGB light with 3 channel analog control
    "IRGB (Analog 4ch Control)",                     # IRGB light with 4 channel analog control
    "Relay (Binary Control with Threshold and Inversion)",  # Relay with binary control including threshold and inversion settings
    "NeoPixel (Digital 1-pin RGB Pixel Control)",    # NeoPixel with digital 1-pin RGB pixel control
    "Servo (PWM 1-pin Servo Motor)"
]

# Settings required for each type of fixture
fixturesSettings = [
    "Address (1-512),Pin",                                   # Settings for Dimmer
    "Address (1-512),Pin (Red),Pin (Blue),Pin (Green)",      # Settings for RGB
    "Address (1-512),Pin (Red),Pin (Blue),Pin (Green)",      # Settings for IRGB
    "Address (1-512),Pin,Threshold (0-256),Inversion (true/false)",  # Settings for Relay
    "Address (1-512),Pin,Number of Cells,Starting Cell",     # Settings for NeoPixel
    "Address (1-512),Pin,Minimum Servo Angle, Maximum Servo Angle"
]

# Fixture classes corresponding to each fixture type
fixtureClasses = [
    "Dimmer",            # Class for Dimmer
    "RGB",               # Class for RGB
    "IRGB",              # Class for IRGB
    "Relay",             # Class for Relay
    "NeoPixel_PM_RGB",   # Class for NeoPixel
    "Servo"
]

# Initialize a 10x1 matrix to store fixtures' settings
fixtures = [
]

# Initialize fixture number counter
fixNum = 0

# Initialize lists for init, setup, and loop sections of the code
init = []
setup = []
loop = []
importer = "#import <OpenFixtures.h>\n"


# Initialize empty string for code generation
code = ""

# Start of the fixture creation program
print("OpenFixtures Fixture Creator (Beta 0.1)")

# Variable to track the current fixture being configured
fix = 0

# Loop through each row in the fixtures matrix
while True:
    print("Select a device from the following options (type END if you are finished):")
    # Display the list of fixture types
    for i in range(len(fixturesDesc)):
        print(str(i+1)+":", fixturesDesc[i])
    
    # Get user input for the selected fixture type
    instruction = input(">  ")
    
    # Check if user wants to end the configuration
    if instruction == "END":
        fixNum += fix  # Update the total number of configured fixtures
        break
    
    # Store the selected fixture type
    fixtures.append([instruction])
    
    # Prompt user for each setting required for the selected fixture type
    for i in range(len(fixturesSettings[int(instruction)-1].split(','))):
        fixtures[fix].append(input(fixturesSettings[int(instruction)-1].split(',')[i]+"?   "))
    
    # Print the current state of fixtures matrix
    print(fixtures)
    
    # Increment fix to move to the next fixture
    fix += 1

# Loop through the configured fixtures and generate init code for each
for i in range(fixNum):
    # Create an initialization string for each fixture
    init.append(fixtureClasses[int(fixtures[i][0])-1]+" Fixture"+str(i)+"("+str(",".join(map(str,fixtures[i][1:])))+");\n")

for i in range(fixNum):
    setup.append("Fixture"+str(i)+".begin();\n")

for i in range(fixNum):
    loop.append("Fixture"+str(i)+".refresh();\n")

# Print the generated initialization code
code = importer+''.join(map(str,init))+ 'void setup() { \n'+''.join(map(str,setup))+'DMXSerial.init(DMXReceiver);\n}\n'+'void loop() {\n'+''.join(map(str,loop))+'}\n'
print(code)