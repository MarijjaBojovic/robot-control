function [ref, dq_ref] = PTP_step(delta_t, tk, startValue, stopValue)
% PTP_STEP - Point-to-Point kretanje sa step promenom reference
% delta_t     - vremenski korak
% tk          - ukupno trajanje simulacije
% startValue  - početna vrednost pozicije [rad]
% stopValue   - krajnja vrednost pozicije [rad]
% ref         - referentna pozicija [2 x N]
% dq_ref      - referentna brzina [2 x N] (sve nule)

N = tk/delta_t;          % broj tačaka
ref = zeros(2, N);
dq_ref = zeros(2, N);

% Step promena
ref(:,1:N) = repmat(stopValue,1,N);  % od početka do kraja je odmah krajnja vrednost
% Početna vrednost (prvi element) može biti startValue
ref(:,1) = startValue;
% Brzina referentno nulirana
dq_ref(:,1:N) = 0;

end
