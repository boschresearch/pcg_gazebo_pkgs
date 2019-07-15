
# pcg_gazebo.parsers
Parsing module to generated and convert SDF, URDF and SDF Configuration formats.

> *Sources*

* [SDF format](http://sdformat.org/)
* [URDF format specifications](https://wiki.ros.org/urdf/XML)


## parse_sdf
```python
parse_sdf(input_xml)
```
Parse an XML file in the SDF format and generates
an `pcg_gazebo` SDF instance.

> *Input arguments*

* `input_xml` (*type:* `str`): Filename of the SDF file or
SDF XML formatted text.

> *Returns*

`pcg_gazebo.parsers.types.XMLBase` object.


## parse_urdf
```python
parse_urdf(input_xml)
```
Parse an XML file in the URDF format and generates
an `pcg_gazebo` URDF instance.

> *Input arguments*

* `input_xml` (*type:* `str`): Filename of the URDF file or
URDF XML formatted text.

> *Returns*

`pcg_gazebo.parsers.types.XMLBase` object.


## parse_sdf_config
```python
parse_sdf_config(input_xml)
```
Parse an XML file in the SDF Configuration format and generates
an `pcg_gazebo` SDF Configuration instance.

> *Input arguments*

* `input_xml` (*type:* `str`): Filename of the SDF Configuration file or
SDF Configuration XML formatted text.

> *Returns*

`pcg_gazebo.parsers.types.XMLBase` object.


## parse_xml
```python
parse_xml(input_xml, type='sdf')
```
Parse an XML file into an `collections.OrderedDict`.

> *Input arguments*

* `input_xml` (*type:* `str`): Filename of the XML file or
XML formatted text.
* `type` (*type:* `str`): Type of XML format used in the input
file, options are `sdf`, `urdf` or `sdf_config`.

> *Returns*

`collections.OrderedDict`: Dictionary where the XML tags are the keys.


## parse_xml_str
```python
parse_xml_str(xml_str, type='sdf')
```
Parse an XML formatted string into an
`collections.OrderedDict`.

> *Input arguments*

* `input_xml` (*type:* `str`): XML formatted text.
* `type` (*type:* `str`): Type of XML format used in the input
file, options are `sdf`, `urdf` or `sdf_config`.

> *Returns*

`collections.OrderedDict`: Dictionary where the XML tags are the keys.


## parse_xml_dict
```python
parse_xml_dict(xml_dict, type='sdf')
```
Converts an `collections.OrderedDict` created from a XML file
and return an SDF, URDF or SDF Configuration `pcg_gazebo` element.

> *Input arguments*

* `xml_dict` (*type:* `collections.OrderedDict`): XML contents.
* `type` (*type:* `str`): Type of XML format used in the input
file, options are `sdf`, `urdf` or `sdf_config`.

> *Returns*

`pcg_gazebo.parsers.types.XMLBase` object.


## convert_to_dict
```python
convert_to_dict(xml_dict)
```
Convert the `xmltodict` output into a dictionary that can be
parsed into a `pcg_gazebo.parsers.types.XMLBase`.

> *Input arguments*

* `xml_dict` (*type:* `collections.OrderedDict`): XML content in
dictionary form.

> *Returns*

`dict`: Formatted XML dictionary.


## convert_from_string
```python
convert_from_string(str_input_xml)
```
Convert a string into a Python data structure type.

> *Input arguments*

* `str_input_xml` (*type:* `str`): Input string

> *Returns*

`bool`, `int`, `float`, list of `float` or `str`.


## sdf2urdf
```python
sdf2urdf(sdf)
```
Recursively convert a SDF `pcg_gazebo` element and its child elements
into an URDF `pcg_gazebo` element.

> *Input arguments*

* `sdf` (*type:* `pcg_gazebo.parsers.types.XMLBase`): Valid SDF element

> *Returns*

`pcg_gazebo.parsers.types.XMLBase` as an URDF element.


## urdf2sdf
```python
urdf2sdf(urdf)
```
Recursively convert an URDF `pcg_gazebo` element and its child elements
into a SDF `pcg_gazebo` element.

> *Input arguments*

* `urdf` (*type:* `pcg_gazebo.parsers.types.XMLBase`): Valid URDF element

> *Returns*

`pcg_gazebo.parsers.types.XMLBase` as a SDF element.

