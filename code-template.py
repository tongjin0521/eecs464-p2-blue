""" Put general module documentation here. It should include:
  -- What does the code in this module do? what key use cases does it support?
     Examples: 
        "this module implements a webserver"; 
        "this modules provides utility classes and functions for making spam"
  -- What are the main classes / functions 
        "the webserver runs from the WebServer class's run() method"
        "clients of this library typically use SpamFactory and SpamEater"  
  -- How is it typically used
     Examples:
        "the webserver is started from the commandline: mywebserver <port>"
        
        "typical usage consists of creating a SpamFactory, generating Spam
         and eating it:
           sf = SpamFactory()
           for spam in sf.generateSpam():
             SpamEater().eat(spam)
        "
  -- What would developers typically subclass/extend?
  -- What other modules does it depend on or support 
"""

class SomeBaseclass( object ):
    """<<<Class documentation goes here>>>
       -- What kind of class is this? e.g. abstract superclass, concrete class, mixin class, function object, etc.
       -- What is this class responsible for / what does it represent?
       -- Who owns instances of this class? i.e. who creates, who destroys?
       -- What is the theory of operation? does it implement a particular algorithm or design pattern? are there any unusual things about how this class works
       -- What are the constraints on how methods are called? e.g. files must be opened before read or written, then closed. Ideally, give a grammar for the usage pattern. For files this is a regular grammar: ((open-for-read)(read)*)|((open-for-write)(write)*)(close)
    """
    def __init__(self, param):
        """<<<constructor documentation goes here>>>
           What do you do to instantiate objects of this class?
           INPUTS: 
             input1 -- <<what kind of object>> -- what is it
           ATTRBIUTES: <<list of *public* attributes >>
             attr1 -- <<what kind of object>> -- what is it for
        """
        pass
        
    def method( self, input1 ):
        """<<<method documentation goes here>>>
           What does the method do? How is it typically used?
           INPUTS:
             input1 -- <<what kind of object>> -- what is it
           OUTPUT:
             output -- <<what kind of object>> -- what is it
           PRECONDITIONS:
             <<what do we assume before being called, e.g. file is already open>>
           POSTCONDITIONS:
             <<what do we guarantee before returning, e.g. file is open or IOException was thrown>>
           THEORY OF OPERATION:
             <<what algorithm / principle does it use to work>>
        """
        ## Conceptual step in the algorithm
        # specific operation to be applied
        # if <<condition>> 
        if condition: # --> <<then action>>
        else: # --> <<else action>>
        # ENDS IF <<condition>> (when if is long enough to be out of screen)
        for var in seq: ## LOOP <<what are we looping for>>
          pass
        # ENDS LOOP <<what are we looping for>> (when out of screen)
        return # (always end with a return - it's easier to read)
        
    def otherMethod( self, input1 ):
        """<<<method documentation goes here>>>
           What does the method do? How is it typically used?
           INPUTS:
             input1 -- <<what kind of object>> -- what is it
           OUTPUT:
             output -- <<what kind of object>> -- what is it
           PRECONDITIONS:
             <<what do we assume before being called, e.g. file is already open>>
           POSTCONDITIONS:
             <<what do we guarantee before returning, e.g. file is open or IOException was thrown>>
           THEORY OF OPERATION:
             <<what algorithm / principle does it use to work>>
        """
        pass
      
class SomeSubclass( SumBaseclass ):
    """
    <<<Class documentation goes here>>>
    -- What kind of class is this? e.g. abstract superclass, concrete class, mixin class, function object, etc.
    -- How is this class different from its baseclass(es)?
    -- What is this class responsible for / what does it represent?
    -- Who owns instances of this class?
    -- What is the theory of operation?
    -- What are the constraints on how methods are called?ose)
    """
    pass

