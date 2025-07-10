"""
Cotton boll criteria to  select the cotton boll with higher grade of interest.
"""

"""
TODO things:

- Should we take into account the depth of the cotton boll when going for one? An interval of values 
  for avoiding going for a cotton boll that's too far maybe?
"""

# Modules
from utils.cotton import Cotton

# Functionalities

# Sorter class
class Sorter(object):
    """
    Sorter class.
    """

    # Class attributes

    # Private methods

    # Public methods

    def __init__(self, data: list = [], rth_cls: int = 0) -> None:
        """
        Constructor of class Sorter. Receives a list of data of the found cotton bolls

        data is a list of dictionaries. Every dict gets:

        - The score (float)
        - The label in image space = numpy.array(xmin, ymin, xmax, ymax)    # TODO tengo que comprobarlo
        - The class (at the moment, 0 if unripe and 1 if ripe)
        """
        self.info: list = []
        """
        Container for every cotton found in given prediction with YOLO. The structure of this structure is as follows:

        {cott_0, cott_1, cott_2, ..., cott_(n-1)}
        
        Given n the number of elements of the list, the 0-th element is the most valuable after sorting, where
        the (n-1)-th element is the least valuable.
        """
        self.rth_cls: int = rth_cls
        """Ready to harvest class number. The rest of classes are not ready to harvest."""

        # TODO quizás sea interesante poner un atributo desired_class donde poner la clase que bsucamos
        # y a partir de ahí poder hacer el filtrado --> También se puede hacer con diccionarios {class_i: cotton_i}

        for d in data:
            score = d["score"]
            label = d["label"]
            cls = d["class"]
            cott = Cotton(score=score, label=label, cls=cls)    # TODO Change parameters
            self.info.append(cott)

        # Sorting criteria
        self.sort_score = lambda x: x.score
        self.sort_cls = lambda x: x.cls

    # Update data

    def update(self, new_info: list = None, new_rth_cls: int = None) -> None:
        """
        Update the data stored in class

        TODO está mal, new_info no contiene [Cotton], contiene [dict]
        """
        if new_info is not None:
            self.info = new_info
        if new_rth_cls is not None:
            self.rth_cls = new_rth_cls
        # More to come

    # Any ready-to-harvest cotton available?

    def any_rth(self) -> bool:
        """
        Return true if there's any cotton boll ready to harvest in current scanning.
        """
        return any(self.info, lambda x: x.cls == self.rth_cls)
    
    # Sort method

    def sort(self, crit: function, reverse: bool = True) -> None:
        """
        Sorting algorithm with selected criteria crit.
        """
        self.info.sort(key=crit, reverse=reverse)   # The highest go the first

    # Filter elements

    def filtr_cls(self, cott: Cotton, cls: int) -> bool:
        """
        Filter criteria by class. To be passed to self.filtr() as function argument
        """
        return cott.cls == cls
    
    def ready_to_harvest(self) -> list:
        """
        Filter all cotton and return a list of only ready-to-harvest cotton
        """
        copy = self.info.copy()
        return list(filter(copy, lambda x: x.cls == self.rth_cls))
    
    # More filters to come

    def filtr(self, f: function) -> None:
        """
        Filter the elements and erase those where f(element) == False
        """
        self.info = list(filter(f, self.info))

    # Pop the best element after sorting

    def pop(self) -> Cotton | None:
        """
        Return the info of the cotton in the beginning of the list.
        """
        if(len(self.info) != 0):
            return self.info.pop(0)
        return None     # No elements to retrieve

