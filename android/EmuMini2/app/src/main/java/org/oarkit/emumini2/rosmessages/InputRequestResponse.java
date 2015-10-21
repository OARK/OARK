package emumini2;

public interface InputRequestResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "emumini2/InputRequestResponse";
  static final java.lang.String _DEFINITION = "Input[] inputs";
  java.util.List<emumini2.Input> getInputs();
  void setInputs(java.util.List<emumini2.Input> value);
}
