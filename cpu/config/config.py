"""
Config loader/saver for DEMETER 
"""

# Generic modules
import json
import numpy as np

# Global parameters
CONFIG_PATH = "./config.json"   # Do not touch! Constant value

# Config class
class Config:
    """
    Config class. Offers a proxy to load and save data from config.json.

    28/08/2024_11:22 -> Class working as expected for configuration loading and saving
    """

    # Private methods
    
    ## Converter
    def __type_convert(self):
        """
        Converts:
        
        - array-like attributes to numpy.ndarray
        """
        ## Convert array-like attributes to numpy arrays (compatible with torch)
        self.data['intrinsic_matrix'] = np.array(self.data['intrinsic_matrix'], dtype=np.float32)
        self.data['distortion_coeffs'] = np.array(self.data['distortion_coeffs'], dtype=np.float32)
        self.data['gripper2cam'] = np.array(self.data['gripper2cam'], dtype=np.float32)

    ## Converter
    def __type_revert(self):
        """
        Converts:
        
        - numpy.arrays into python lists
        """
        ## Convert array-like attributes to numpy arrays (compatible with torch)
        self.data['intrinsic_matrix'] = self.data['intrinsic_matrix'].tolist()
        self.data['distortion_coeffs'] = self.data['distortion_coeffs'].tolist()
        self.data['gripper2cam'] = self.data['gripper2cam'].tolist()

    # Public methods

    ## Constructor
    def __init__(self):
        self.data = {}
        self.load()
    
    ## Loader / Saver

    def load(self):
        """Load configuration data from the JSON file."""
        with open(CONFIG_PATH, 'r') as file:
            self.data = json.load(file)
        ## Convert array-like structures to Numpy arrays
        self.__type_convert()

    def save(self):
        """Save the current configuration data to the JSON file."""
        # Reconvert to JSON friendly format
        self.__type_revert()
        # Save
        with open(CONFIG_PATH, 'w') as file:
            json.dump(self.data, file, indent=4)
        #Convert to numpy types again
        self.__type_convert()

    ## Getter
    def get(self, key):
        """Retrieve a specific parameter from the configuration data."""
        if key in self.data.keys():
            return self.data.get(key)
        else:
            raise KeyError("No key has been found in Config data dictionary")
    
    ## Setter
    def set(self, key, value):
        """Set a specific parameter in the configuration data."""
        self.data[key] = value