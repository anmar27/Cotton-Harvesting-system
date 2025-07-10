"""
Representation, loading and storage methods of the representation of the cotton in world space.

We'll use Databases or CSV to represent the state of the world in every moment of consideration. Every table (or combination of tables)
represents a moment where the state has been saved. This could be every day, every hour, every minute... whatever. The naming for every
database file will be:

    YYYY_MM_DD:HH_MM_SS.db      # First MM means month, second MM means minutes, db generic database extension to be changed (.csv, .sql, ...)

- At the moment we'll use comma separated values (csv) but we could change to sql databases if needed

TODO:

    - We'll use databases to store the world representation --> python datasets (tables) 
        · To future encrypt the database, we'll use https://docs.devart.com/python/sqlite/database-encryption.htm (optional)

    - Maybe we need to use a property decorator everytime we change the dataframe, to keep track of consistency

    - Como la información viene de los algodones, tenemos que tener claro lo que queremos tener en cuenta al guardar el estado de un algodón
    determinado, de manera que podemos mover esa info entre el world state y otros módulos
"""

# Generic modules
import os
import pandas as pd
from datetime import datetime

# Utils
from ..utils.cotton import Cotton   # TODO this has to be changed from utils to world for state representation

# Constants

# World state class

class World(object):
    """
    Represents the world state at given point in time. World stores the values of detected cotton in self.db as a dataframe, to be stored in
    csv, database, ontology... format.
    """

    def __init__(self, saves_path: str, cols: list):
        """
        Initialization of the world representation. Creates an empty world representation.
        """
        self.id: str = datetime.now().strftime("%y_%m_%d:%H_%M_%S")         # Name of the current state
        self.is_cur_in_file: bool = False                                   # Is the current state in file?
        self.is_consistent: bool = True                                     # Is the table consistent with file system?
        
        self.saves_path: str = saves_path                                   # Path to world savings

        self.db: pd.DataFrame = pd.DataFrame(                               # Dataset with current world state
            columns=cols        # ['id','x','y','z','cat'] Are some examples, but have to be passed through parameter
        )                              
        """
        Database were we store all data related to the cotton bolls, the state they're in, labels...

        The database has the following structure:

        TABLE:
            [NOT] id: int --> This will be the INDEX of every entry!
            x: float
            y: float
            z: float
            cat: float --> Corresponding to the categories in imageset (YOLO)
            score: float
            
            [TODO] average_days_to_following_state: int --> we need to keep in track the average time to change
        """
        return
    
    # Element getter

    def get(self, cott_id: int) -> Cotton:
        """
        Retrieves the data about a given cotton with identifier cott_id
        """
        cott_info = self.db.loc[cott_id]

        # Translate from cotton info to Cotton object

        cott = Cotton() # TODO
        return cott
    
    # Element setter (change values)

    def modify(self, info: Cotton) -> bool:
        """
        Modifies the given entry with existing id in database.

        parameter: info, contains the information to be changed. Every element must be some sort of attribute in dataframe
        """

        # TODO

        pass
        
    # Element adder
    
    def add(self, info: Cotton) -> None:
        """
        Add an entry to actual database. The parameter given has to have all column attributes set

        TODO
        """
        cott_info = None

        # Translate the info in Cotton to cott_info

        self.db.loc[len(self.db)] = cott_info
    
    # State loading

    def load_cur_state(self, db_name: str, db_path: str) -> None:
        """
        Load from files the current state of the world.
        """
        # CSV
        self.id = db_name
        self.is_cur_in_file = True
        self.db = pd.read_csv(f"{db_path}/{db_name}")
        
        """
        # SQL

        conn = sqlite3.connect(db_path)
        self.db = pd.read_sql('SELECT * FROM world_state', conn)        # Keep in mind the name of the table!
        """

    # State saving
    
    def save_cur_state(self) -> None:
        """
        Save the current representation of the world in selected format.

        Suppose that the world save is done at world/world_save and a directory is created for every save (we can save other files there too)
        """
        # Create directory if doesn't exist
        saving_path = f"{self.saves_path}/{self.id}"
        os.makedirs(saving_path, exist_ok=True)

        # CSV
        self.db.to_csv(f'{saving_path}.csv', index=False)

        """
        # SQL
        
        More complicated, code from the beginning if SQL desired
        """















