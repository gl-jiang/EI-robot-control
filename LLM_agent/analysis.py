"""
@author: Yutang Li
"""
import time
import random
import pandas as pd
from openai import OpenAI, APIError, RateLimitError


TABLE_PATH = "data/experiment_data.xlsx"
_MODELS = ["claude-3-5-sonnet-latest", "gpt-4.1", "deepseek-chat", "qwen-max-latest", "gemini-2.5-pro-exp-03-25", "claude-3-7-sonnet-20250219-thinking", "o3", "o4-mini", "deepseek-reasoner"]

prompt = """
Please conduct a factor analysis of the experimental results based on the following table (which contains experimental record data). The table records the experimental results (first column) and six experimental factors, including: Injection position, Injection speed, Cooling route, Reaction temperature, Cooling temperature, and Cs-OL temperature. Each factor is set at multiple levels, and the experiment is designed using the controlled variable method.

Please:
1. Carefully analyze the influence of each factor on the experimental results in the first column.
2. Provide a detailed impact analysis for each factor (explaining why and how the factor affects the results, combined with data variations).
3. Quantify the importance of each factor on a scale of 1-5 (5 being the most important, 1 being the least important) and justify the scoring.
4. Present the analysis results in a structured format.

Note: Your response should be specifically based on the experimental data, avoiding generalized discussions.

{table}
"""

def generate_analysis_result():
    # read a excel file
    table = pd.read_excel(TABLE_PATH, sheet_name=0)
    # convert the table to markdown format
    table_markdown = table.to_markdown(index=True)
    # add the table to the prompt
    prompt_with_table = prompt.format(table=table_markdown)
    # generate the analysis result
    messages = [{"role": "user", "content": prompt_with_table}]
    # print(messages)
    for model in _MODELS:
        print(f"model: {model}")
        # get the response from the model
        content = get_response_from_llm(messages, model_name=model)
        # print the content
        print(content)
        # save the content to a file
        with open(f"data/experiment_analysis_{model}.txt", "w") as f:
            f.write(content)


def get_response_from_llm(messages: list[dict], model_name: str, tools: list = None, max_retries: int = 3, initial_backoff: float = 1.0):
    """
    Get response from LLM API with retry mechanism.
    
    Args:
        messages: List of message dictionaries
        model_name: Name of the model to use
        tools: Optional list of tools to use
        max_retries: Maximum number of retry attempts
        initial_backoff: Initial backoff time in seconds
    
    Returns:
        Content of the response or error message
    """
    retries = 0
    while retries <= max_retries:
        try:
            client = OpenAI(api_key="sk-oYh3Xrhg8oDY2gW02c966f31C84449Ad86F9Cd9dF6E64a8d", base_url="https://vip.apiyi.com/v1")
            # messages = [{"role": "user", "content": "9.11 and 9.8, which is greater?"}]
            if tools is None:
                response = client.chat.completions.create(
                    model=model_name,
                    messages=messages,
                )
            else:
                response = client.chat.completions.create(
                    model=model_name,
                    messages=messages,
                    tools=tools,
                    tool_choice='auto',
                    parallel_tool_calls=True
                )
            content = response.choices[0].message.content
            return content
        
        except RateLimitError as rate_error:
            retries += 1
            if retries > max_retries:
                print(f"Max retries exceeded for RateLimitError: {rate_error}")
                return 'apierror'
            
            # Exponential backoff with jitter
            backoff_time = initial_backoff * (2 ** (retries - 1)) * (0.5 + random.random())
            print(f"Rate limit hit, retrying in {backoff_time:.2f} seconds (attempt {retries}/{max_retries})")
            time.sleep(backoff_time)
        
        except APIError as api_error:
            retries += 1
            if retries > max_retries:
                print(f"Max retries exceeded for APIError: {api_error}")
                return 'apierror'
            
            # Check if the error is retryable
            error_str = str(api_error)
            if "timeout" in error_str.lower() or "connection" in error_str.lower() or "server" in error_str.lower():
                # Exponential backoff with jitter
                backoff_time = initial_backoff * (2 ** (retries - 1)) * (0.5 + random.random())
                print(f"API error, retrying in {backoff_time:.2f} seconds (attempt {retries}/{max_retries}): {api_error}")
                time.sleep(backoff_time)
            else:
                # Non-retryable API error
                print(f"Non-retryable API error: {api_error}")
                return 'apierror'
        
        except Exception as e:
            print(f"generate_design_question Unexpected error: {e}")
            return 'unexpectederror'


def get_response_from_deepseek_r1(messages: list[dict], prefix: bool = False, max_retries: int = 3, initial_backoff: float = 1.0):
    """
    Get response from DeepSeek API with retry mechanism.
    
    Args:
        messages: List of message dictionaries
        prefix: Whether to use the prefix URL
        max_retries: Maximum number of retry attempts
        initial_backoff: Initial backoff time in seconds
    
    Returns:
        Tuple of (reasoning_content, content) or error messages
    """
    retries = 0
    while retries <= max_retries:
        try:
            base_url = "https://api.deepseek.com/beta" if prefix else "https://vip.apiyi.com/v1"
            api_key = "sk-59279cc16ec740089146ef9aef9c1671" if prefix else "sk-oYh3Xrhg8oDY2gW02c966f31C84449Ad86F9Cd9dF6E64a8d"

            client = OpenAI(api_key=api_key, base_url=base_url)
            # messages = [{"role": "user", "content": "9.11 and 9.8, which is greater?"}]

            response = client.chat.completions.create(
                model="deepseek-r1",
                messages=messages,
                temperature=0.6
            )

            # reasoning_content = "null" if prefix else "<think>\n" + response.choices[0].message.model_extra['reasoning_content'] + "\n</think>\n"
            reasoning_content = response.choices[0].message.content.split("</think>\n")[0].split("<think>\n")[-1]
            content = response.choices[0].message.content.split("</think>\n")[-1]
            return reasoning_content, content
        
        except RateLimitError as rate_error:
            retries += 1
            if retries > max_retries:
                print(f"Max retries exceeded for RateLimitError: {rate_error}")
                return 'apierror', 'apierror'
            
            # Exponential backoff with jitter
            backoff_time = initial_backoff * (2 ** (retries - 1)) * (0.5 + random.random())
            print(f"Rate limit hit, retrying in {backoff_time:.2f} seconds (attempt {retries}/{max_retries})")
            time.sleep(backoff_time)
        
        except APIError as api_error:
            retries += 1
            if retries > max_retries:
                print(f"Max retries exceeded for APIError: {api_error}")
                return 'apierror', 'apierror'
            
            # Check if the error is retryable
            error_str = str(api_error)
            if "timeout" in error_str.lower() or "connection" in error_str.lower() or "server" in error_str.lower():
                # Exponential backoff with jitter
                backoff_time = initial_backoff * (2 ** (retries - 1)) * (0.5 + random.random())
                print(f"API error, retrying in {backoff_time:.2f} seconds (attempt {retries}/{max_retries}): {api_error}")
                time.sleep(backoff_time)
            else:
                # Non-retryable API error
                print(f"Non-retryable API error: {api_error}")
                return 'apierror', 'apierror'
        
        except Exception as e:
            print(f"generate_design_question Unexpected error: {e}")
            return 'unexpectederror', 'unexpectederror'