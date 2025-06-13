from langchain.tools import tool
import threading

_current_global_vision_context = "No Visual Information available"
_vision_context_lock = threading.Lock() # For thread-safe updates

def set_global_vision_context(context: str):
    """Updates the global vision context."""
    global _current_global_vision_context
    with _vision_context_lock:
        _current_global_vision_context = context

def get_global_vision_context() -> str:
    """Retrieves the global vision context."""
    global _current_global_vision_context
    with _vision_context_lock:
        return _current_global_vision_context

@tool
def vision_tool() -> str:
    """
    Tool to process and analyze vision context from the VLM model.
    This tool retrieves the latest global vision context available.
    It takes no arguments, fetching the context from a shared source.

    Returns:
        str: Analyzed response based on the vision context.
    """
    vision_context = get_global_vision_context() # Get from the global variable

    if not vision_context or vision_context == "No Visual Information available":
        return "I don't have any visual information available right now."

    try:
        # Extract key information from structured vision context
        lines = vision_context.split('\n')
        summary = ""
        env_type = ""
        action = ""

        for line in lines:
            if "Scene summary:" in line:
                summary = line.split("Scene summary:", 1)[1].strip()
            elif "Environment type:" in line:
                env_type = line.split("Environment type:", 1)[1].strip()
            elif "Suggested action:" in line:
                action = line.split("Suggested action:", 1)[1].strip()

        # Provide a descriptive string for the LLM
        return f"I can see that we're in a {env_type}. {summary} Based on what I see, {action}"
    except Exception as e:
        # Log or handle parsing errors gracefully
        return f"I can see something, but I'm having trouble analyzing it in detail: {vision_context}"