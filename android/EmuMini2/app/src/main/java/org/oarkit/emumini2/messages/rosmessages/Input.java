package org.oarkit.emumini2.messages.rosmessages;

public interface Input extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "org/oarkit/emumini2/messages/rosmessages/Input";
  static final java.lang.String _DEFINITION = "string name\nstring type\nstring title\nstring axes\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getType();
  void setType(java.lang.String value);
  java.lang.String getTitle();
  void setTitle(java.lang.String value);
  java.lang.String getAxes();
  void setAxes(java.lang.String value);
}
