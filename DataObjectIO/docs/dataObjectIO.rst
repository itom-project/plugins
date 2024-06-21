===================
 DataObjectIO
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`DataObjectIO`
**Type**:       :plugintype:`DataObjectIO`
**License**:    :pluginlicense:`DataObjectIO`
**Platforms**:  Windows, Linux
**Author**:     :pluginauthor:`DataObjectIO`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: DataObjectIO

These algorithms are defined in the plugin:

.. pluginfilterlist::
    :plugin: DataObjectIO
    :overviewonly:

.. py:function:: send_message(sender, recipient, message_body, [priority=1])

   Send a message to a recipient

   :param str sender: The person sending the message
   :param str recipient: The recipient of the message
   :param str message_body: The body of the message
   :param priority: The priority of the message, can be a number 1-5
   :type priority: int or None
   :return: the message id
   :rtype: int
   :raises ValueError: if the message_body exceeds 160 characters
   :raises TypeError: if the message_body is not a basestring

Details
==============

Detailed overview about all defined algorithms:

.. pluginfilterlist::
    :plugin: DataObjectIO

Changelog
==========

* itom setup 1.2.0: Release
