# !/bin/bash

# Ask if the user wants to set up a virtual environment
read -p "Do you want to set up a virtual environment? (y/n) "
if [[ "$REPLY" == "y" ]]; then
    # Create the virtual environment
    python3 -m venv .venv
    # Activate the virtual environment
    source .venv/bin/activate
    # Install the required packages
    pip install -r requirements.txt
fi

# Run the sys id script with the different bag files
echo "Torpedo Vehicle System Identification with LS Without Noise"
python3 sysid_ls.py data/coug_sim.npz

echo "Torpedo Vehicle System Identification with LS With Noise"
python3 sysid_ls.py data/coug_sim.npz --noisy

echo "BlueROV2 System Identification with WLS Without Noise"
python3 sysid_wls.py data/bluerov2_no_noise

echo "BlueROV2 System Identification with WLS With Noise"
python3 sysid_wls.py data/bluerov2_noise

echo "Real World BlueROV2 System Identification with WLS With Noise"
python3 sysid_wls.py data/rb_x_1.0-2026-02-19-08-31-13