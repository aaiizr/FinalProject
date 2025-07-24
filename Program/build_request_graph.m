function G = build_request_graph(sent)
        N=numel(sent); G = cell(N,1);
        for k=1:N, G{k}=sent(k).to; end
    end