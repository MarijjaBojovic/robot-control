function Ps = write_in_memory(Ps, i, dt, q, dq, X, X_ref, q_ref, dq_ref, ddq_ref, Tau, Fint)
%WRITE_IN_MEMORY Zapisuje podatke simulacije u strukturu Ps
%
% Ps           - struktura koja se popunjava
% i            - indeks trenutnog koraka
% dt           - vremenski korak
% q, dq        - stvarne pozicije i brzine zglobova (2x1)
% X, X_ref     - pozicija efektora i referentna (2x1)
% q_ref, dq_ref, ddq_ref - referentna pozicija, brzina i ubrzanje zglobova (2x1)
% Tau          - momenti zglobova (2x1)
% Fint         - interaktivna sila (2x1)

    % Zapis vremena
    Ps.t(i) = i * dt;
    
    % Stvarne pozicije i brzine
    Ps.q(:,i)  = q .* (180/pi);
    Ps.dq(:,i) = dq .* (180/pi);
    
    % Kartiške koordinate
    Ps.X(:,i)     = X;
    Ps.X_ref(:,i) = X_ref;
    
    % Referentne pozicije, brzine i ubrzanja
    Ps.q_ref(:,i)  = q_ref .* (180/pi);
    Ps.dq_ref(:,i) = dq_ref .* (180/pi);
    Ps.ddq_ref(:,i)= ddq_ref;
    
    % Moment i sila
    Ps.Tau(:,i)  = Tau;
    Ps.Fint(:,i) = Fint;
end
