function spec = reqCreateSpec(id, nl, pattern, nvargs)
%REQCREATESPEC Create a requirement specification struct for rendering.
%
%   spec = reqCreateSpec(ID, NL, PATTERN, Name=Value)
%
%   Creates a structured specification that can be passed to rendering
%   functions (reqRenderStructuredEnglish, reqRenderToRT, reqRenderToTA).
%
%   Required Arguments:
%     ID      - Requirement ID string, e.g. 'TSM-R01'
%     NL      - Original natural language text
%     PATTERN - Pattern classification string:
%               'Invariant', 'BoundedOutput', 'TriggerResponse',
%               'Persistence', 'DurationLimit', 'ModeDependent',
%               'Latching', 'Initialization', 'Summary'
%
%   Name-Value Arguments:
%     Guard            - Precondition expression (MATLAB syntax).
%                        Used for Invariant, BoundedOutput, ModeDependent.
%                        Example: 'FC == 0'
%
%     Property         - Postcondition expression (MATLAB syntax).
%                        Used for Invariant, BoundedOutput, ModeDependent.
%                        Example: 'sel_val == median(ia, ib, ic)'
%
%     TriggerType      - 'level' | 'rising_edge' | 'falling_edge'
%                        Default: 'level'
%
%     TriggerExpr      - Trigger expression (MATLAB syntax).
%                        Example: 'enable == 1'
%
%     TriggerDuration  - Duration (seconds) for persistent trigger.
%                        When set, trigger means "becomes true AND stays
%                        true for at least this many seconds."
%
%     ResponseType     - 'must_be_true' | 'hold_at_least' | 'hold_at_most'
%                        | 'hold_between' | 'weak_until'
%                        Default: 'must_be_true'
%
%     ResponseExpr     - Response expression (MATLAB syntax).
%                        Example: 'ready == true'
%
%     ResponseDuration - Duration(s) for hold_* responses.
%                        Scalar for hold_at_least/hold_at_most.
%                        [T1 T2] for hold_between.
%
%     UntilExpr        - Condition expression for weak_until response.
%                        Example: 'reset_cmd == 1'
%
%     DelayType        - 'none' | 'at_most' | 'between'
%                        Default: 'none'
%
%     DelayValue       - Delay value (seconds).
%                        Scalar for 'at_most'. [a b] for 'between'.
%
%     InitProperty     - Property that must hold at t=0 (MATLAB syntax).
%                        Used for 'Initialization' pattern.
%
%     Confidence       - 'HIGH' | 'MEDIUM' | 'LOW' | 'AMBIGUOUS'
%                        Default: 'HIGH'
%
%     Variables        - Struct mapping NL terms to model signals.
%                        Example: struct('y','sel_val','u1','ia')
%
%     Description      - Short human-readable summary of the requirement.
%
%   Examples:
%     % Invariant: output equals median when no faults
%     spec = reqCreateSpec('TSM-R01', ...
%         'The system output shall equal the median of the inputs.', ...
%         'Invariant', ...
%         Guard='FC == 0', ...
%         Property='sel_val == median(ia, ib, ic)', ...
%         Confidence='HIGH', ...
%         Variables=struct('y','sel_val','u1','ia','u2','ib','u3','ic'));
%
%     % Trigger-response: brake disengages cruise within 0.5s
%     spec = reqCreateSpec('CC-R07', ...
%         'When brake is pressed, cruise shall disengage within 0.5s.', ...
%         'TriggerResponse', ...
%         TriggerType='level', ...
%         TriggerExpr='brake_pressed == 1', ...
%         ResponseType='must_be_true', ...
%         ResponseExpr='cruise_engaged == 0', ...
%         DelayType='at_most', ...
%         DelayValue=0.5);
%
%     % Initialization
%     spec = reqCreateSpec('TSM-R02.1', ...
%         'Sensor status shall be initialized to 1.', ...
%         'Initialization', ...
%         InitProperty='s1 == 1 && s2 == 1 && s3 == 1');
%
%   See also reqRenderStructuredEnglish, reqCheckRenderability,
%            reqRenderToRT, reqRenderToTA

    arguments
        id      (1,1) string
        nl      (1,1) string
        pattern (1,1) string {mustBeMember(pattern, ...
            {'Invariant','BoundedOutput','TriggerResponse', ...
             'Persistence','DurationLimit','ModeDependent', ...
             'Latching','Initialization','Summary'})}
        nvargs.Guard            (1,1) string = ""
        nvargs.Property         (1,1) string = ""
        nvargs.TriggerType      (1,1) string {mustBeMember(nvargs.TriggerType, ...
            {'level','rising_edge','falling_edge'})} = "level"
        nvargs.TriggerExpr      (1,1) string = ""
        nvargs.TriggerDuration  double = []
        nvargs.ResponseType     (1,1) string {mustBeMember(nvargs.ResponseType, ...
            {'must_be_true','hold_at_least','hold_at_most', ...
             'hold_between','weak_until'})} = "must_be_true"
        nvargs.ResponseExpr     (1,1) string = ""
        nvargs.ResponseDuration double = []
        nvargs.UntilExpr        (1,1) string = ""
        nvargs.DelayType        (1,1) string {mustBeMember(nvargs.DelayType, ...
            {'none','at_most','between'})} = "none"
        nvargs.DelayValue       double = []
        nvargs.InitProperty     (1,1) string = ""
        nvargs.Confidence       (1,1) string {mustBeMember(nvargs.Confidence, ...
            {'HIGH','MEDIUM','LOW','AMBIGUOUS'})} = "HIGH"
        nvargs.Variables        struct = struct()
        nvargs.Description      (1,1) string = ""
    end

    spec.id          = id;
    spec.nl          = nl;
    spec.pattern     = pattern;
    spec.confidence  = nvargs.Confidence;
    spec.description = nvargs.Description;
    spec.variables   = nvargs.Variables;

    % Guard / Property (for Invariant, BoundedOutput, ModeDependent)
    spec.guard    = nvargs.Guard;
    spec.property = nvargs.Property;

    % Trigger (for TriggerResponse, Persistence, DurationLimit, Latching)
    spec.trigger.type     = nvargs.TriggerType;
    spec.trigger.expr     = nvargs.TriggerExpr;
    spec.trigger.duration = nvargs.TriggerDuration;

    % Response
    spec.response.type     = nvargs.ResponseType;
    spec.response.expr     = nvargs.ResponseExpr;
    spec.response.duration = nvargs.ResponseDuration;
    spec.response.untilExpr = nvargs.UntilExpr;

    % Delay
    spec.delay.type  = nvargs.DelayType;
    spec.delay.value = nvargs.DelayValue;

    % Initialization
    spec.initProperty = nvargs.InitProperty;

    % Validate pattern-specific required fields
    switch pattern
        case {'Invariant', 'BoundedOutput', 'ModeDependent'}
            if spec.property == ""
                error('reqCreateSpec:MissingField', ...
                    'Pattern "%s" requires the Property argument.', pattern);
            end
        case {'TriggerResponse', 'Persistence', 'DurationLimit'}
            if spec.trigger.expr == ""
                error('reqCreateSpec:MissingField', ...
                    'Pattern "%s" requires the TriggerExpr argument.', pattern);
            end
            if spec.response.expr == ""
                error('reqCreateSpec:MissingField', ...
                    'Pattern "%s" requires the ResponseExpr argument.', pattern);
            end
        case 'Latching'
            if spec.trigger.expr == ""
                error('reqCreateSpec:MissingField', ...
                    'Pattern "Latching" requires the TriggerExpr argument.');
            end
            if spec.response.expr == ""
                spec.response.expr = spec.trigger.expr;
            end
        case 'Initialization'
            if spec.initProperty == ""
                error('reqCreateSpec:MissingField', ...
                    'Pattern "Initialization" requires the InitProperty argument.');
            end
    end
end
