function polyFunc = createPolyFunction(coefficients)
    % This function creates a polynomial function based on the given degree
    % and coefficients, without using polyval.
    %
    % Args:
    %     degree (integer): The degree of the polynomial.
    %     coefficients (array): Coefficients of the polynomial from the highest degree term to the constant term.
    %
    % Returns:
    %     polyFunc (function handle): A handle to the polynomial function that evaluates the polynomial at any x.

    degree = length(coefficients); 

    % Create the polynomial function handle
    polyFunc = @(x) sum(coefficients .* (x .^ (degree:-1:0)));
end