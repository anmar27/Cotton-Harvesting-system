# Modules
import numpy as np
import uuid

# Cotton class

class Cotton(object):
    """
    Cotton class. Contains every useful info bout a cotton boll detection. Used for sorting.

    TODO if we use the attribute names to create the columns of the database, every type has to be simple,
    not structured (no lists, tuples, dicts, np.arrays...)
    """

    # Class attributes

    _ID_GEN: int = np.nan      # Has to be set before instantiating any cotton object
    """
    Generator of cotton identifiers. This attribute can have value in the range of positive integers [0, inf)
    """

    # Private methods

    def __get_id(self) -> None:
        """
        Get next id for a given cotton object. This function maintains consistency so that every identifier is unique. 
        
        _ID_GEN has to be set before any cotton instantiation.
        """
        id = Cotton._ID_GEN
        Cotton._ID_GEN = Cotton._ID_GEN + 1
        return id
    
    # Public methods

    ## ID generator setter
    def set_ID_gen(self, id_gen: int) -> None:
        """
        Setter of the generator of unique identifiers. Has to be called before instantiating the first cotton object.

        ID generator in range [0,inf)
        """
        if type(id_gen) == int and id_gen >= 0: Cotton._ID_GEN = id_gen
        else: raise Exception("No valid value for ID generator")

    ## Constructor
    def __init__(self, cls: int, label: np.ndarray, score: float, x: float, y: float, z: float) -> None:
        """Constructor of class Cotton."""

        # Is ID generator set?
        if Cotton._ID_GEN == np.nan:
            raise Exception("Generator has not been set yet")
        
        # Constructor
        self.id: int = self.__get_id() # TODO tengo que lidiar con esto más tarde (cómo los generamos?)

        self.cls: int = cls             # Classes are represented as integers
        self.label: np.ndarray = label  # Label = box (xmin,ymin,xmax,ymax)
        self.score: float = score

        self.x: float = x
        self.y: float = y
        self.z: float = z

    # Getter
    def to_dict(self) -> dict:
        """
        Returns all information in object in the form of dictionary {attr_name: attr_value}
        """
        return self.__dict__


    