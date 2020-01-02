from ckbot.logical import Cluster

"""
Unlike most of the other demos in this directory, demo-clp is primarily text-file documentation of the CLP functionality in ckbot.logical.Cluster. As such, it sits in the infrastructure _under_ JoyApp applications. 

What is a CLP?
==============

A CLP (pronounced CLaP) is a "CLuster Property", i.e. a readable or writable property addressable in a Cluster. Depending on whether it is readable or writable, the Cluster will provide a getter function, a setter function, or both for the property. Similar to URL-s, CLP-s can follow one of several schemes:

(1) Hardware scheme, formatted as XX:YYYY, with XX and YYYY being hexadecimal digits. This scheme gets mapped directly into Robotics Bus Object Dictionary entry YYYY on node XX. Read and write permissions are mapped automatically from the object dictionary. As an example, "21:1051" is OD entry 0x1051 of node number 0x21 (33 decimal); on a standard CKBot servo module, this would be the "encpos" property that reads the servo encoder position.

The hardware scheme is there to allow code to address experimental OD entries that do not yet have a sensible, human readable name.

(2) OD Property scheme, formatted as node/property, "node" being the node name as appearing the Cluster's .at member, and "property" being the Object Dictionary property name, as mapped by cfg/od_names.yml. As an example, if node 0x21 of the previous example was called "Banana", the same property could be accessed via "Banana/encpos". These properties only become available once the node's Object Dictionary was walked (scanned through), e.g. using .get_od().

(3) Attribute scheme, formatted as node/@attribute, "node" being the node name as appearing the Cluster's .at member, and "property" being a property exposed by the designer of the Module subclass that the node belongs to. While the OD properties require the host to walk (scan through) the object dictionary of the node before they become available, Module classes have the option of exposing functions that either take a 16bit int as an input or ones that return a 16bit int. In the ServoModule classes, these include set_pos, get_pos. Thus, in the attribute scheme, the same property could be accessed via "Banana/@get_pos", except that it would not require the "Banana" module's OD to have been walked before becoming available.


Implementing Attribute CLP-s
============================

Attribute CLP-s are implemented in two parts:

(1) A method with the same name as the CLP must be added to the Module. The method may either take (self,val) and return None, val being a 16bit int, or it may take (self) and return a 16bit int.

(2) The attributes must be registered in a dictionary stored in the _attr member of the Module, e.g.
>>> m._attr=dict(  set_pos="2W", is_slack="1R" )
sets up two attributes, a writable set_pos(int) and a readable is_slack().


Scanning for CLP-s
==================

The CLP-s of a Cluster and of a Module are indexed. On Modules, the iterhwattr() iterator can be used to scan through the hardware addressing scheme CLP-s; the iterprop() iterator gives the OD scheme CLP-s; and the iterattr() iterator gives the attribute CLP-s.
On Cluster-s, the iterhwaddr() iterator will scan through all OD entries of all modules (which has been walked), in order.
The iterprop() iterator will scan through attribute CLP-s (if enabled) and through OD CLP-s.


Using CLP-s
===========

Clusters have factory methods .getterOf(clp) and .setterOf(clp) that return getter and setter functions for the CLP.
"""

def getAllOD( clst ):
  """
  Get all readable object dictionary properties in a Cluster.
  
  INPUT:
    clst -- a Cluster
    
  OUTPUT:
    dictionary with Hardware CLP mapping to value
  """
  # Make sure all OD-s were walked
  for node in clst.itervalues():
    if node.od is None:
      node.get_od()
  res = {}
  # Scan CLP-s
  for clp in clst.iterhwaddr():
    get = clst.getterOf(clp)
    res[clp] = get()
  return res

def getDofByNID( clst ):
  """
  Get an array of position setting functions for all servo Modules, 
  sorted in order of increasing node-id.
  
  INPUT:
    clst -- a Cluster
    
  OUTPUT:
    list of setter functions
  
  NOTE: this does not require walking the OD-s
  """
  res = []
  for node in clst.itervalues():
    try:
      s = clst.setterOf("%s/@set_pos" % node.name)
    except KeyError as ke:
      print(ke)
      continue
    res.append((node.node_id,s))
  res.sort()
  return res

