# selfie_app_communication

## Usage

### Parameters

### Description

This package contains the server end of the application used to:
change parameters using dynamic_reconfigure
be able to monitor the values of important variables
be able to send flags, call servers etc..

#### Settings
the application can be easily configured just by changing the settings.xml file.
#### Message format
![message_format](https://github.com/KNR-Selfie/selfie_carolocup2020/blob/app/ramka.png)
the message consists of two parts: 
first is a unique code for every exchanged variable, it consists of 3 bytes.
second is a value, it can have a variable length, depending on the type of a message exchanged. The lengths are as stated in struct package [documentation](https://docs.python.org/2/library/struct.html).

